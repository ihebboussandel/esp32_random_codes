# 🌀 Fast AHT10 Sensor Reader using I2C on Custom Pins

This Arduino sketch provides a fast and minimal implementation to communicate with the **AHT10 temperature and humidity sensor** using I2C on **custom `SDA`/`SCL` pins**. It uses manual command/control to trigger measurements and read raw data directly.

## 🧰 Features

- Manual command interface for the AHT10
- Uses `Wire.begin(SDA, SCL)` to support boards like ESP32 or others with custom pin support
- Simple parsing of raw temperature and humidity data
- Soft-reset at startup (optional, improves reliability)
- Prints data with microsecond timing for performance evaluation

---

## 🔧 Wiring

| AHT10 Pin | Arduino Pin |
|----------|-------------|
| SDA      | `D8`        |
| SCL      | `D9`        |
| VCC      | `3.3V/5V`   |
| GND      | `GND`       |

> ⚠️ Make sure your board supports `Wire.begin(SDA, SCL)` syntax (e.g. **ESP32**, **ESP8266**, etc). For AVR (UNO, Nano), use hardware SDA/SCL only.

---

## 📟 Output Format

Example Serial Monitor output:
Fast AHT10 I2C Read
Temp: 24.57 °C Hum: 55.63 %RH Took: 9.8 ms
Temp: 24.60 °C Hum: 55.70 %RH Took: 9.8 ms

## 📈 Performance
- Sensor read time: ~9.8 ms
- Read rate: Up to 10 Hz

##Tested On
- AHT10 I2C sensor module
- ESP32-S3 
- Arduino IDE 2.x

## 📚 References

- [AHT10 Datasheet (PDF)](https://aosong.com/userfiles/files/media/aht10-datasheet.pdf)
- [Arduino Wire Library Documentation](https://www.arduino.cc/en/Reference/Wire)
