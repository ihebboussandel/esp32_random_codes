#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "thermistor.h"
#include "HardwareSerial.h"

// OLED setup
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(-1);

// Thermistor setup
#define NTC_PIN 2
THERMISTOR thermistor(NTC_PIN, 11000, 3950, 10000);
uint16_t temp; // Temp in 1/10 °C

void setup() {
  Wire.begin(5, 4); // SDA, SCL pins for ESP8266
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  Serial.begin(115200);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();
  delay(2000);
  display.clearDisplay();
}

void loop() {
  temp = thermistor.read(); // Read temp in 1/10 ºC
  float tempC = temp / 10.0; // Convert to float

  Serial.print("Temp in 1/10 ºC : ");
  Serial.println(temp);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("Temp: ");
  display.print(tempC, 1); // Show 1 decimal place
  display.println(" C");

  display.display();
  //delay(1000); // Slower refresh for OLED
}
