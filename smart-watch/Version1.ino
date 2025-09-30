/*LVGL-based RTOS Sensor System for Waveshare ESP32-S3-LCD-1.28
 *Integrates QMI8658 sensor data with WiFi transmission and circular GUI
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

// QMI8658 6-axis IMU sensor I2C address and registers
#define QMI8658_I2C_ADDR 0x6B
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL7 0x08
#define QMI8658_ACC_X_L 0x35
#define QMI8658_GYRO_X_L 0x3B
#define QMI8658_TEMP_L 0x33

// Hardware pin definitions
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

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
static bool wifi_connected = false;
static bool qmi8658_initialized = false;
static sensor_data_t latest_sensor_data;

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
void init_qmi8658_sensor();
void update_ui_data(sensor_data_t *data, time_data_t *time_data);
sensor_data_t read_qmi8658_data();
bool qmi8658_write_reg(uint8_t reg, uint8_t data);
uint8_t qmi8658_read_reg(uint8_t reg);
void qmi8658_read_multiple(uint8_t reg, uint8_t *data, uint8_t len);
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

    // Initialize system time
    struct timeval tv = {.tv_sec = 1725465600, .tv_usec = 0};
    settimeofday(&tv, NULL);
    setenv("TZ", "UTC", 1);
    tzset();

    // Initialize latest sensor data
    memset(&latest_sensor_data, 0, sizeof(latest_sensor_data));

    // Create queues and semaphores
    sensor_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data_t));
    wifi_queue = xQueueCreate(WIFI_QUEUE_SIZE, sizeof(sensor_data_t));
    display_mutex = xSemaphoreCreateMutex();

    if (!sensor_queue || !wifi_queue || !display_mutex) {
        ESP_LOGE(TAG, "Failed to create queues or semaphores");
        return;
    }

    // Initialize peripherals
    init_qmi8658_sensor();
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
        sensor_data = read_qmi8658_data();
        
        // Update global latest data
        latest_sensor_data = sensor_data;
        
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
            // Get current time
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
    
    // Configure NTP
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "UTC", 1);  // Set your timezone: "IST-5:30" for India
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
    ESP_LOGI(TAG, "Time synchronized: %02d:%02d:%02d %02d/%02d/%04d", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
             timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    
    while (1) {
        // Resync every 24 hours
        vTaskDelay(pdMS_TO_TICKS(24 * 60 * 60 * 1000));
        configTime(0, 0, "pool.ntp.org");
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
        
        server.begin();
        ESP_LOGI(TAG, "Web server started at http://%s", WiFi.localIP().toString().c_str());
    } else {
        wifi_connected = false;
        ESP_LOGE(TAG, "WiFi connection failed");
    }
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Sensor Hub</title>";
    html += "<style>body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0}";
    html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".sensor-box{background:#f8f9fa;padding:15px;margin:10px 0;border-radius:5px;border-left:4px solid #007bff}";
    html += "h1{color:#333;text-align:center}";
    html += ".refresh{background:#007bff;color:white;padding:10px 20px;border:none;border-radius:5px;cursor:pointer;margin:10px 0}";
    html += ".refresh:hover{background:#0056b3}</style>";
    html += "<meta http-equiv='refresh' content='5'></head><body>";
    html += "<div class='container'>";
    html += "<h1>🌟 ESP32-S3 Sensor Hub 🌟</h1>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>🌡️ Temperature</h3>";
    html += "<p>" + String(latest_sensor_data.temperature, 2) + " °C</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>📱 Accelerometer</h3>";
    html += "<p>X: " + String(latest_sensor_data.accel_x, 3) + " m/s²</p>";
    html += "<p>Y: " + String(latest_sensor_data.accel_y, 3) + " m/s²</p>";
    html += "<p>Z: " + String(latest_sensor_data.accel_z, 3) + " m/s²</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>🎯 Gyroscope</h3>";
    html += "<p>X: " + String(latest_sensor_data.gyro_x, 2) + " °/s</p>";
    html += "<p>Y: " + String(latest_sensor_data.gyro_y, 2) + " °/s</p>";
    html += "<p>Z: " + String(latest_sensor_data.gyro_z, 2) + " °/s</p>";
    html += "</div>";
    
    html += "<div class='sensor-box'>";
    html += "<h3>🔋 Battery</h3>";
    html += "<p>" + String(latest_sensor_data.battery_voltage / 1000.0, 2) + " V</p>";
    html += "</div>";
    
    html += "<p style='text-align:center;margin-top:30px'>";
    html += "<a href='/data' style='text-decoration:none'>";
    html += "<button class='refresh'>📊 Get JSON Data</button></a>";
    html += "</p>";
    
    html += "<p style='text-align:center;color:#666;font-size:12px'>Auto-refreshes every 5 seconds</p>";
    html += "</div></body></html>";
    
    server.send(200, "text/html", html);
}

void handleSensorData() {
    String json = "{";
    json += "\"timestamp\":" + String(latest_sensor_data.timestamp) + ",";
    json += "\"temperature\":" + String(latest_sensor_data.temperature, 2) + ",";
    json += "\"accelerometer\":{";
    json += "\"x\":" + String(latest_sensor_data.accel_x, 3) + ",";
    json += "\"y\":" + String(latest_sensor_data.accel_y, 3) + ",";
    json += "\"z\":" + String(latest_sensor_data.accel_z, 3);
    json += "},";
    json += "\"gyroscope\":{";
    json += "\"x\":" + String(latest_sensor_data.gyro_x, 2) + ",";
    json += "\"y\":" + String(latest_sensor_data.gyro_y, 2) + ",";
    json += "\"z\":" + String(latest_sensor_data.gyro_z, 2);
    json += "},";
    json += "\"battery_voltage\":" + String(latest_sensor_data.battery_voltage);
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

// QMI8658 sensor functions
void init_qmi8658_sensor() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t who_am_i = qmi8658_read_reg(QMI8658_WHO_AM_I);
    ESP_LOGI(TAG, "QMI8658 WHO_AM_I: 0x%02X", who_am_i);
    
    if (who_am_i == 0x05) {
        qmi8658_write_reg(QMI8658_CTRL1, 0x60);
        qmi8658_write_reg(QMI8658_CTRL2, 0x60);
        qmi8658_write_reg(QMI8658_CTRL3, 0x12);
        qmi8658_write_reg(QMI8658_CTRL7, 0x00);
        qmi8658_initialized = true;
        ESP_LOGI(TAG, "QMI8658 initialized successfully");
    } else {
        qmi8658_initialized = false;
        ESP_LOGE(TAG, "QMI8658 not found");
    }
}

sensor_data_t read_qmi8658_data() {
    sensor_data_t data;
    memset(&data, 0, sizeof(data));
    
    if (!qmi8658_initialized) {
        // Simulate sensor data if sensor is not available
        data.accel_x = 0.1f * sin(millis() / 1000.0f);
        data.accel_y = 0.1f * cos(millis() / 1000.0f);
        data.accel_z = 9.8f + 0.1f * sin(millis() / 2000.0f);
        data.gyro_x = 1.0f * sin(millis() / 3000.0f);
        data.gyro_y = 1.0f * cos(millis() / 3000.0f);
        data.gyro_z = 0.5f * sin(millis() / 4000.0f);
        data.temperature = 25.0f + 2.0f * sin(millis() / 10000.0f);
        data.battery_voltage = 3700 + (esp_random() % 600);
        time(&data.timestamp);
        return data;
    }
    
    uint8_t sensor_data_raw[12];
    int16_t acc_raw[3], gyro_raw[3], temp_raw;
    
    qmi8658_read_multiple(QMI8658_ACC_X_L, sensor_data_raw, 6);
    acc_raw[0] = (int16_t)(sensor_data_raw[1] << 8 | sensor_data_raw[0]);
    acc_raw[1] = (int16_t)(sensor_data_raw[3] << 8 | sensor_data_raw[2]);
    acc_raw[2] = (int16_t)(sensor_data_raw[5] << 8 | sensor_data_raw[4]);
    
    qmi8658_read_multiple(QMI8658_GYRO_X_L, sensor_data_raw, 6);
    gyro_raw[0] = (int16_t)(sensor_data_raw[1] << 8 | sensor_data_raw[0]);
    gyro_raw[1] = (int16_t)(sensor_data_raw[3] << 8 | sensor_data_raw[2]);
    gyro_raw[2] = (int16_t)(sensor_data_raw[5] << 8 | sensor_data_raw[4]);
    
    qmi8658_read_multiple(QMI8658_TEMP_L, sensor_data_raw, 2);
    temp_raw = (int16_t)(sensor_data_raw[1] << 8 | sensor_data_raw[0]);
    
    data.accel_x = (float)acc_raw[0] * 4.0f * 9.80665f / 32768.0f;
    data.accel_y = (float)acc_raw[1] * 4.0f * 9.80665f / 32768.0f;
    data.accel_z = (float)acc_raw[2] * 4.0f * 9.80665f / 32768.0f;
    
    data.gyro_x = (float)gyro_raw[0] * 512.0f / 32768.0f;
    data.gyro_y = (float)gyro_raw[1] * 512.0f / 32768.0f;
    data.gyro_z = (float)gyro_raw[2] * 512.0f / 32768.0f;
    
    data.temperature = (float)temp_raw / 256.0f;
    data.battery_voltage = 3700 + (esp_random() % 600);
    time(&data.timestamp);
    
    return data;
}

bool qmi8658_write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(QMI8658_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

uint8_t qmi8658_read_reg(uint8_t reg) {
    Wire.beginTransmission(QMI8658_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)QMI8658_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void qmi8658_read_multiple(uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(QMI8658_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)QMI8658_I2C_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }
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