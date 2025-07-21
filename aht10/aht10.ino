#include <Wire.h>

#define SDA_PIN 8     // Your actual SDA pin
#define SCL_PIN 9     // Your actual SCL pin
#define AHT10_ADDR 0x38

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Fast AHT10 I2C Read");

  // Soft reset (optional, helps some sensors)
  Wire.beginTransmission(AHT10_ADDR);
  Wire.write(0xBA);
  Wire.endTransmission();
  delay(20);
}

void loop() {
  unsigned long startMicros = micros();

  // Trigger measurement: 0xAC 0x33 0x00
  Wire.beginTransmission(AHT10_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(80);  // Wait minimum recommended time

  // Read 6 bytes from AHT10
  Wire.requestFrom(AHT10_ADDR, 6);
  if (Wire.available() == 6) {
    uint8_t raw[6];
    for (int i = 0; i < 6; i++) raw[i] = Wire.read();

    // Parse humidity (20-bit)
    uint32_t hum_raw = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | (raw[3] >> 4);
    float humidity = hum_raw * 100.0 / 1048576.0;

    // Parse temperature (20-bit)
    uint32_t temp_raw = ((uint32_t)(raw[3] & 0x0F) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
    float temperature = temp_raw * 200.0 / 1048576.0 - 50.0;

    // Timing
    unsigned long elapsedMicros = micros() - startMicros;

    // Print results
    Serial.print("Temp: "); Serial.print(temperature, 2); Serial.print(" °C\t");
    Serial.print("Hum: "); Serial.print(humidity, 2); Serial.print(" %RH\t");
    Serial.print("Took: "); Serial.print(elapsedMicros / 1000.0, 1); Serial.println(" ms");
  } else {
    Serial.println("Sensor read error");
  }

  // Very short delay to hit ~10 Hz max read rate
  delay(10);
}
