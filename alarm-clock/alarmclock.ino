#include <Wire.h> //for taling to I2C, RTC, LCD
#include <RTClib.h> //handling real time clock functions
#include <LiquidCrystal_I2C.h> //handling I2C LCD

RTC_DS3231 rtc; //creating RTC object
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows

const int buzzerPin = 9; //connection to buzzer D9
// DateTime nowManual(2025, 7, 30, 20, 44, 50);  // YYYY,MM,DD,HH,MM,SS

void setup() {
  Serial.begin(9600); //printing messages to serial monitor
  Wire.begin(); //starting I2C communication
  lcd.begin(16, 2); //starting LCD with size 16x2
  lcd.backlight(); //turn on LCD light

  if (!rtc.begin()) {
    lcd.print("RTC not found!");
    while (1); //just freeze if RTC's missing
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting default time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // setting RTC to compile time
    // rtc.adjust(nowManual); // You can use this to manually hardcode a datetime
      }

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

void loop() { //this loop always does checking time & updating the LCD.
  DateTime now = rtc.now(); //getting time from RTC

  char timeBuffer[9];
  sprintf(timeBuffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  char dateBuffer[11];
  sprintf(dateBuffer, "%02d-%02d-%04d", now.day(), now.month(), now.year());

//display on LCD
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(timeBuffer);

  lcd.setCursor(0, 1);
  lcd.print("Date: ");
  lcd.print(dateBuffer);

  // Alarm condition for now i have set as one for 9:30 PM (21:30:00)
  if (now.hour() == 21 && now.minute() == 31 && now.second() == 0) {
    tone(buzzerPin, 1000);  // Buzzer on
  } else {
    noTone(buzzerPin);      // Buzzer off
  }

  delay(1000); // waiting one second before updating again!
}
