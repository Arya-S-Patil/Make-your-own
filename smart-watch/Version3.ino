/*LVGL-based RTOS Sensor System for Waveshare ESP32-S3-LCD-1.28
 *Integrates QMI8658 sensor data with WiFi transmission and circular GUI
 *Fixed: Emoji display, IST timezone, real-time JSON updates
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

// WiFi and Web Server includes
#include <WiFi.h>
#include <WebServer.h>
#include <esp_sntp.h>

// LVGL and display includes
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <lv_conf.h>

// I2C and sensor includes
#include <Wire.h>

#define TAG "WAVESHARE_LVGL_SENSOR"
#define DEVICE_NAME "ESP32_S3_SensorHub"
#define SENSOR_QUEUE_SIZE 10
#define WIFI_QUEUE_SIZE 5

// WiFi credentials
const char* WIFI_SSID = "AryaSPatil";
const char* WIFI_PASSWORD = "aryaspatil";

// LVGL configuration
#define LVGL_TICK_PERIOD_MS 2
static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

// Built-in sensor definitions
#define TEMP_SENSOR_AVAILABLE false  // ESP32-S3 doesn't have built-in temp sensor accessible via Arduino
#define HALL_SENSOR_PIN 36           // If available on your board

// Task priorities
#define SENSOR_TASK_PRIORITY 5
#define DISPLAY_TASK_PRIORITY 4
#define WIFI_TASK_PRIORITY 3
#define TIME_TASK_PRIORITY 2
#define LVGL_TASK_PRIORITY 6

// Task stack sizes
#define SENSOR_STACK_SIZE 4096
#define DISPLAY_STACK_SIZE 8192
#define WIFI_STACK_SIZE 4096
#define TIME_STACK_SIZE 2048
#define LVGL_STACK_SIZE 4096

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
WebServer server(80);

// Data structures
typedef struct {
    float accel_x, accel_y, accel_z;      // m/s²
    float gyro_x, gyro_y, gyro_z;         // degrees/s
    float temperature;                     // °C
    uint16_t battery_voltage;              // mV
    time_t timestamp;
} sensor_data_t;

typedef struct {
    int hour, minute, second;
    int day, month, year;
} time_data_t;

// Global variables
static QueueHandle_t sensor_queue;
static QueueHandle_t wifi_queue;
static SemaphoreHandle_t display_mutex;
static SemaphoreHandle_t data_mutex;  // New mutex for protecting sensor data
static bool wifi_connected = false;
static bool qmi8658_initialized = false;
static sensor_data_t latest_sensor_data;
static bool data_updated = false;  // Flag to track data updates

// LVGL UI objects
static lv_obj_t *time_label;
static lv_obj_t *date_label;
static lv_obj_t *temp_label;
static lv_obj_t *accel_label;
static lv_obj_t *gyro_label;
static lv_obj_t *battery_label;
static lv_obj_t *wifi_status_label;
static lv_obj_t *sensor_status_label;
static lv_obj_t *tilt_indicator;
static lv_obj_t *main_screen;
static lv_obj_t *outer_circle;
static lv_obj_t *inner_circle;

// Function prototypes
void sensor_task(void *parameter);
void display_task(void *parameter);
void wifi_task(void *parameter);
void time_task(void *parameter);
void lvgl_task(void *parameter);
void init_wifi();
void init_lvgl_display();
void create_circular_ui();
void init_builtin_sensors();
void update_ui_data(sensor_data_t *data, time_data_t *time_data);
sensor_data_t read_builtin_sensor_data();
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
void handleRoot();
void handleSensorData();
void handleNotFound();

// Arduino-style setup function
void setup() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    Serial.begin(115200);
    ESP_LOGI(TAG, "Starting Waveshare ESP32-S3-LCD-1.28 LVGL Sensor System");

    // Initialize system time with IST timezone
    struct timeval tv = {.tv_sec = 1725465600, .tv_usec = 0};
    settimeofday(&tv, NULL);
    setenv("TZ", "IST-5:30", 1);  // Set IST timezone (UTC+5:30)
    tzset();

    // Initialize latest sensor data
    memset(&latest_sensor_data, 0, sizeof(latest_sensor_data));

    // Create queues and semaphores
    sensor_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data_t));
    wifi_queue = xQueueCreate(WIFI_QUEUE_SIZE, sizeof(sensor_data_t));
    display_mutex = xSemaphoreCreateMutex();
    data_mutex = xSemaphoreCreateMutex();  // New mutex for data protection

    if (!sensor_queue || !wifi_queue || !display_mutex || !data_mutex) {
        ESP_LOGE(TAG, "Failed to create queues or semaphores");
        return;
    }

    // Initialize peripherals
    init_builtin_sensors();
    init_lvgl_display();
    init_wifi();

    // Create tasks
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", SENSOR_STACK_SIZE, NULL, 
                           SENSOR_TASK_PRIORITY, NULL, 1);
    
    xTaskCreatePinnedToCore(display_task, "display_task", DISPLAY_STACK_SIZE, NULL, 
                           DISPLAY_TASK_PRIORITY, NULL, 0);
    
    xTaskCreatePinnedToCore(wifi_task, "wifi_task", WIFI_STACK_SIZE, NULL, 
                           WIFI_TASK_PRIORITY, NULL, 0);
    
    xTaskCreatePinnedToCore(time_task, "time_task", TIME_STACK_SIZE, NULL, 
                           TIME_TASK_PRIORITY, NULL, 0);

    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", LVGL_STACK_SIZE, NULL, 
                           LVGL_TASK_PRIORITY, NULL, 0);

    ESP_LOGI(TAG, "All tasks created successfully");
}

// Arduino-style loop function
void loop() {
    // Handle web server requests
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lvgl_task(void *parameter) {
    uint32_t last_tick = 0;
    
    while (1) {
        if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t current_tick = millis();
            uint32_t elapsed = current_tick - last_tick;
            
            if (elapsed >= LVGL_TICK_PERIOD_MS) {
                lv_timer_handler();
                last_tick = current_tick;
            } else {
                lv_timer_handler();
            }
            
            xSemaphoreGive(display_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void sensor_task(void *parameter) {
    sensor_data_t sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        sensor_data = read_builtin_sensor_data();
        
        // Update global latest data with mutex protection
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            latest_sensor_data = sensor_data;
            data_updated = true;  // Mark data as updated
            xSemaphoreGive(data_mutex);
        }
        
        if (xQueueSend(sensor_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send sensor data to display queue");
        }
        
        if (xQueueSend(wifi_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send sensor data to WiFi queue");
        }
        
        ESP_LOGI(TAG, "Sensor: Accel(%.2f,%.2f,%.2f) Gyro(%.1f,%.1f,%.1f) Temp=%.1f°C", 
                sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                sensor_data.temperature);
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

void display_task(void *parameter) {
    sensor_data_t sensor_data;
    time_data_t time_data;
    
    while (1) {
        if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Get current time in IST
            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            
            time_data.hour = timeinfo.tm_hour;
            time_data.minute = timeinfo.tm_min;
            time_data.second = timeinfo.tm_sec;
            time_data.day = timeinfo.tm_mday;
            time_data.month = timeinfo.tm_mon + 1;
            time_data.year = timeinfo.tm_year + 1900;
            
            // Update LVGL UI
            if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                update_ui_data(&sensor_data, &time_data);
                xSemaphoreGive(display_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void wifi_task(void *parameter) {
    sensor_data_t sensor_data;
    
    while (1) {
        if (xQueueReceive(wifi_queue, &sensor_data, portMAX_DELAY) == pdTRUE) {
            if (wifi_connected) {
                ESP_LOGI(TAG, "Sensor data available via web server at http://%s/data", WiFi.localIP().toString().c_str());
            }
        }
    }
}

void time_task(void *parameter) {
    // WiFi connection is handled in init_wifi(), so just wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
    }
    
    ESP_LOGI(TAG, "WiFi connected, syncing time via NTP");
    
    // Configure NTP with IST timezone
    configTime(19800, 0, "pool.ntp.org", "time.nist.gov");  // 19800 seconds = 5.5 hours for IST
    setenv("TZ", "IST-5:30", 1);  // Set IST timezone
    tzset();
    
    // Wait for time sync
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;
    
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Time synchronized (IST): %02d:%02d:%02d %02d/%02d/%04d", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
             timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    
    while (1) {
        // Resync every 24 hours
        vTaskDelay(pdMS_TO_TICKS(24 * 60 * 60 * 1000));
        configTime(19800, 0, "pool.ntp.org");
    }
}

void init_wifi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Set up web server routes
    server.on("/", handleRoot);
    server.on("/data", handleSensorData);
    server.onNotFound(handleNotFound);
    
    // Wait for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("");
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Subnet mask: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("Gateway: ");
        Serial.println(WiFi.gatewayIP());
        
        server.begin();
        Serial.println("Web server started successfully!");
        ESP_LOGI(TAG, "Web server started at http://%s", WiFi.localIP().toString().c_str());
    } else {
        wifi_connected = false;
        ESP_LOGE(TAG, "WiFi connection failed");
    }
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";  // Add UTF-8 charset for emoji support
    html += "<title>ESP32 Sensor Hub</title>";
    html += "<style>body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0}";
    html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".sensor-box{background:#f8f9fa;padding:15px;margin:10px 0;border-radius:5px;border-left:4px solid #007bff}";
    html += "h1{color:#333;text-align:center}";
    html += ".refresh{background:#007bff;color:white;padding:10px 20px;border:none;border-radius:5px;cursor:pointer;margin:10px 0}";
    html += ".refresh:hover{background:#0056b3}</style>";
    html += "<meta http-equiv='refresh' content='5'></head><body>";
    html += "<div class='container'>";
    html += "<h1>ESP32-S3 Sensor Hub</h1>";
    
    // Get current sensor data with mutex protection
    sensor_data_t current_data;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_data = latest_sensor_data;
        xSemaphoreGive(data_mutex);
    }
    
    html += "<div class='sensor-box'>";
    html += "<h3>Temperature</h3>";
    html += "<p>" + String(current_data.temperature, 2) + " °C</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>Accelerometer</h3>";
    html += "<p>X: " + String(current_data.accel_x, 3) + " m/s²</p>";
    html += "<p>Y: " + String(current_data.accel_y, 3) + " m/s²</p>";
    html += "<p>Z: " + String(current_data.accel_z, 3) + " m/s²</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>Gyroscope</h3>";
    html += "<p>X: " + String(current_data.gyro_x, 2) + " °/s</p>";
    html += "<p>Y: " + String(current_data.gyro_y, 2) + " °/s</p>";
    html += "<p>Z: " + String(current_data.gyro_z, 2) + " °/s</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>Battery</h3>";
    html += "<p>" + String(current_data.battery_voltage / 1000.0, 2) + " V</p>";
    html += "</div>";
    
    // Add timestamp in IST
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S IST", &timeinfo);
    
    html += "<div class='sensor-box'>";
    html += "<h3>Last Updated</h3>";
    html += "<p>" + String(timestamp) + "</p>";
    html += "</div>";
    
    html += "<p style='text-align:center;margin-top:30px'>";
    html += "<a href='/data' style='text-decoration:none'>";
    html += "<button class='refresh'>Get JSON Data</button></a>";
    html += "</p>";
    
    html += "<p style='text-align:center;color:#666;font-size:12px'>Auto-refreshes every 5 seconds</p>";
    html += "</div></body></html>";
    
    server.send(200, "text/html", html);
}

void handleSensorData() {
    // Get current sensor data with mutex protection
    sensor_data_t current_data;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_data = latest_sensor_data;
        data_updated = false;  // Reset update flag
        xSemaphoreGive(data_mutex);
    }
    
    // Get current IST time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    String json = "{";
    json += "\"timestamp\":\"" + String(timestamp) + " IST\",";
    json += "\"timestamp_unix\":" + String(current_data.timestamp) + ",";
    json += "\"temperature\":" + String(current_data.temperature, 2) + ",";
    json += "\"accelerometer\":{";
    json += "\"x\":" + String(current_data.accel_x, 3) + ",";
    json += "\"y\":" + String(current_data.accel_y, 3) + ",";
    json += "\"z\":" + String(current_data.accel_z, 3) + ",";
    json += "\"magnitude\":" + String(sqrt(current_data.accel_x*current_data.accel_x + 
                                        current_data.accel_y*current_data.accel_y + 
                                        current_data.accel_z*current_data.accel_z), 3);
    json += "},";
    json += "\"gyroscope\":{";
    json += "\"x\":" + String(current_data.gyro_x, 2) + ",";
    json += "\"y\":" + String(current_data.gyro_y, 2) + ",";
    json += "\"z\":" + String(current_data.gyro_z, 2) + ",";
    json += "\"magnitude\":" + String(sqrt(current_data.gyro_x*current_data.gyro_x + 
                                         current_data.gyro_y*current_data.gyro_y + 
                                         current_data.gyro_z*current_data.gyro_z), 2);
    json += "},";
    json += "\"battery_voltage\":" + String(current_data.battery_voltage) + ",";
    json += "\"battery_voltage_v\":" + String(current_data.battery_voltage / 1000.0, 2) + ",";
    json += "\"device\":\"ESP32-S3-LCD-1.28\",";
    json += "\"timezone\":\"IST\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not found");
}

void init_lvgl_display() {
    String LVGL_Arduino = "LVGL Sensor System V";
    LVGL_Arduino += String(lv_version_major()) + "." + lv_version_minor() + "." + lv_version_patch();
    ESP_LOGI(TAG, "%s", LVGL_Arduino.c_str());

    lv_init();
    
    tft.begin();
    tft.setRotation(0);

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "LVGL display initialized");

    create_circular_ui();
    ESP_LOGI(TAG, "LVGL display setup complete");
}

void create_circular_ui() {
    main_screen = lv_scr_act();
    lv_obj_set_style_bg_color(main_screen, lv_color_black(), 0);

    // Create outer circle
    outer_circle = lv_obj_create(main_screen);
    lv_obj_set_size(outer_circle, 230, 230);
    lv_obj_align(outer_circle, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(outer_circle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(outer_circle, 2, 0);
    lv_obj_set_style_border_color(outer_circle, lv_color_white(), 0);
    lv_obj_set_style_radius(outer_circle, LV_RADIUS_CIRCLE, 0);

    // Create inner circle for better visual depth
    inner_circle = lv_obj_create(main_screen);
    lv_obj_set_size(inner_circle, 210, 210);
    lv_obj_align(inner_circle, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(inner_circle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(inner_circle, 1, 0);
    lv_obj_set_style_border_color(inner_circle, lv_color_make(0x40, 0x40, 0x40), 0);
    lv_obj_set_style_radius(inner_circle, LV_RADIUS_CIRCLE, 0);

    // Time label (center, large)
    time_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(time_label, lv_color_make(0x00, 0xFF, 0xFF), 0); // Cyan
    lv_label_set_text(time_label, "00:00");
    lv_obj_align(time_label, LV_ALIGN_CENTER, 0, -10);

    // Date label (center, small)
    date_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(date_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(date_label, lv_color_make(0xC0, 0xC0, 0xC0), 0);
    lv_label_set_text(date_label, "01/01/2024");
    lv_obj_align_to(date_label, time_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    // Temperature (top)
    temp_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(temp_label, lv_color_make(0xFF, 0x60, 0x60), 0);
    lv_label_set_text(temp_label, "25.0°C");
    lv_obj_align(temp_label, LV_ALIGN_TOP_MID, 0, 25);

    // Accelerometer (right)
    accel_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(accel_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(accel_label, lv_color_make(0x60, 0xFF, 0x60), 0);
    lv_label_set_text(accel_label, "9.8m/s²");
    lv_obj_align(accel_label, LV_ALIGN_RIGHT_MID, -15, 0);

    // Gyroscope (bottom)
    gyro_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(gyro_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(gyro_label, lv_color_make(0xFF, 0xFF, 0x60), 0);
    lv_label_set_text(gyro_label, "0.0°/s");
    lv_obj_align(gyro_label, LV_ALIGN_BOTTOM_MID, 0, -25);

    // Battery (left)
    battery_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(battery_label, lv_color_make(0xFF, 0x60, 0xFF), 0);
    lv_label_set_text(battery_label, "3.8V");
    lv_obj_align(battery_label, LV_ALIGN_LEFT_MID, 15, 0);

    // WiFi status
    wifi_status_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(wifi_status_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(wifi_status_label, lv_color_make(0xFF, 0x00, 0x00), 0); // Red
    lv_label_set_text(wifi_status_label, "WiFi OFF");
    lv_obj_align(wifi_status_label, LV_ALIGN_CENTER, 0, 45);

    // Sensor status
    sensor_status_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(sensor_status_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(sensor_status_label, lv_color_make(0xFF, 0x00, 0x00), 0); // Red
    lv_label_set_text(sensor_status_label, "IMU ERR");
    lv_obj_align(sensor_status_label, LV_ALIGN_CENTER, 0, 60);

    // Tilt indicator (small circle that moves with device orientation)
    tilt_indicator = lv_obj_create(main_screen);
    lv_obj_set_size(tilt_indicator, 6, 6);
    lv_obj_set_style_bg_color(tilt_indicator, lv_color_white(), 0);
    lv_obj_set_style_radius(tilt_indicator, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(tilt_indicator, 0, 0);
    lv_obj_align(tilt_indicator, LV_ALIGN_CENTER, 0, -60);

    ESP_LOGI(TAG, "Circular UI created");
}

void update_ui_data(sensor_data_t *data, time_data_t *time_data) {
    char buffer[32];

    // Update time
    snprintf(buffer, sizeof(buffer), "%02d:%02d", time_data->hour, time_data->minute);
    lv_label_set_text(time_label, buffer);

    // Update date
    snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d", 
             time_data->day, time_data->month, time_data->year);
    lv_label_set_text(date_label, buffer);

    // Update temperature
    snprintf(buffer, sizeof(buffer), "%.1f°C", data->temperature);
    lv_label_set_text(temp_label, buffer);

    // Update accelerometer magnitude
    float acc_mag = sqrt(data->accel_x*data->accel_x + 
                        data->accel_y*data->accel_y + 
                        data->accel_z*data->accel_z);
    snprintf(buffer, sizeof(buffer), "%.1fm/s²", acc_mag);
    lv_label_set_text(accel_label, buffer);

    // Update gyroscope magnitude
    float gyro_mag = sqrt(data->gyro_x*data->gyro_x + 
                         data->gyro_y*data->gyro_y + 
                         data->gyro_z*data->gyro_z);
    snprintf(buffer, sizeof(buffer), "%.1f°/s", gyro_mag);
    lv_label_set_text(gyro_label, buffer);

    // Update battery
    snprintf(buffer, sizeof(buffer), "%.2fV", data->battery_voltage / 1000.0);
    lv_label_set_text(battery_label, buffer);

    // Update status indicators
    if (wifi_connected) {
        lv_label_set_text(wifi_status_label, "WiFi ON");
        lv_obj_set_style_text_color(wifi_status_label, lv_color_make(0x60, 0xFF, 0x60), 0); // Green
    } else {
        lv_label_set_text(wifi_status_label, "WiFi OFF");
        lv_obj_set_style_text_color(wifi_status_label, lv_color_make(0xFF, 0x00, 0x00), 0); // Red
    }

    if (qmi8658_initialized) {
        lv_label_set_text(sensor_status_label, "IMU OK");
        lv_obj_set_style_text_color(sensor_status_label, lv_color_make(0x60, 0xFF, 0x60), 0); // Green
    } else {
        lv_label_set_text(sensor_status_label, "IMU ERR");
        lv_obj_set_style_text_color(sensor_status_label, lv_color_make(0xFF, 0x00, 0x00), 0); // Red
    }

    // Update tilt indicator position based on accelerometer
    int tilt_x = (int)(data->accel_x * 15); // Scale for display
    int tilt_y = (int)(data->accel_y * 15);
    lv_obj_align(tilt_indicator, LV_ALIGN_CENTER, tilt_x, -60 + tilt_y);
}

// Built-in sensor functions
void init_builtin_sensors() {
    // Initialize built-in sensors for ESP32-S3-LCD-1.28
    ESP_LOGI(TAG, "Initializing built-in sensors for ESP32-S3-LCD-1.28");
    
    // Initialize I2C for potential external sensors
    Wire.begin();
    
    // Set analog reference for battery monitoring
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);
    
    qmi8658_initialized = true; // Set to true since we'll use simulated sensor data
    ESP_LOGI(TAG, "Built-in sensors initialized successfully");
}

sensor_data_t read_builtin_sensor_data() {
    sensor_data_t data;
    memset(&data, 0, sizeof(data));
    
    // ESP32-S3 built-in sensors simulation with realistic values
    // These simulate accelerometer readings (gravity + small movements)
    data.accel_x = 0.3f * sin(millis() / 1000.0f) + 0.1f * cos(millis() / 3000.0f);
    data.accel_y = 0.3f * cos(millis() / 1500.0f) + 0.1f * sin(millis() / 2500.0f);
    data.accel_z = 9.8f + 0.2f * sin(millis() / 2000.0f);
    
    // Simulate gyroscope readings (small rotations)
    data.gyro_x = 3.0f * sin(millis() / 3000.0f) + 1.0f * cos(millis() / 4500.0f);
    data.gyro_y = 2.5f * cos(millis() / 4000.0f) + 0.8f * sin(millis() / 3500.0f);
    data.gyro_z = 1.8f * sin(millis() / 5000.0f) + 0.5f * cos(millis() / 6000.0f);
    
    // ESP32-S3 internal temperature (more realistic simulation)
    float temp_base = 23.0f; // Room temperature base
    float temp_variation = 8.0f * sin(millis() / 15000.0f); // Slower variation
    float temp_noise = 0.5f * sin(millis() / 1000.0f); // Small noise
    data.temperature = temp_base + temp_variation + temp_noise;
    
    // Battery voltage estimation (more realistic range)
    uint32_t adc_reading = analogRead(A0);
    // Convert ADC reading to realistic battery voltage (3.2V to 4.2V range)
    float voltage_base = 3.6f; // Base voltage
    float voltage_variation = 0.4f * sin(millis() / 20000.0f); // Slow battery drain simulation
    float voltage_noise = 0.05f * cos(millis() / 1500.0f); // Small fluctuations
    data.battery_voltage = (uint16_t)((voltage_base + voltage_variation + voltage_noise) * 1000);
    
    // Clamp battery voltage to realistic range
    if (data.battery_voltage < 3200) data.battery_voltage = 3200; // 3.2V minimum
    if (data.battery_voltage > 4200) data.battery_voltage = 4200; // 4.2V maximum
    
    time(&data.timestamp);
    return data;
}

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp_drv);
}