# 🌡️ Thermistor OLED Temperature Display

This project reads temperature data from an NTC thermistor and displays it on a 128x64 SSD1306 OLED display using I2C. Designed for Arduino-compatible boards like ESP8266 or ESP32.

---


---

## 🔧 Features
- Real-time temperature reading
- OLED graphical display (SSD1306 128x64)
- High-precision readings using thermistor equations
- Serial output for debugging

---

## 📦 Hardware Requirements
- ESP8266 / ESP32 / Arduino Uno / Nano
- NTC thermistor (e.g. 10k 3950)
- 10k resistor (for voltage divider)
- SSD1306 I2C OLED display
- Breadboard + jumper wires

---

## 🔌 Wiring Example (ESP8266)
| Component   | ESP8266 Pin |
|-------------|-------------|
| Thermistor voltage divider output | A0 (Analog pin) |
| OLED SDA    | D1 (GPIO5)   |
| OLED SCL    | D2 (GPIO4)   |
| GND         | GND          |
| VCC         | 3.3V         |

---

## 📄 Code Overview

```cpp
temp = thermistor.read();   // Read temperature in 1/10 °C
float tempC = temp / 10.0;  // Convert to °C
display.print(tempC);       // Show on OLED
