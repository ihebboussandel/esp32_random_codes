#include <Wire.h>

#define AHT10_ADDRESS 0x38
#define ENS160_ADDRESS 0x53
#define SDA_PIN 4
#define SCL_PIN 5

// ENS160 registers
#define ENS160_REG_PART_ID       0x00
#define ENS160_REG_OPMODE        0x10
#define ENS160_REG_CONFIG        0x11
#define ENS160_REG_COMMAND       0x12
#define ENS160_REG_TEMP_IN       0x13
#define ENS160_REG_RH_IN         0x15
#define ENS160_REG_DEVICE_STATUS 0x20
#define ENS160_REG_DATA_AQI      0x21
#define ENS160_REG_DATA_TVOC     0x22
#define ENS160_REG_DATA_ECO2     0x24
#define ENS160_REG_DATA_T        0x30
#define ENS160_REG_DATA_RH       0x32
#define ENS160_REG_DATA_MISR     0x38
#define ENS160_REG_GPR_READ      0x48  // 8 bytes

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("AHT10 + ENS160 Register Read + Compensation Demo");

  // ---- Initialize AHT10 ----
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(0xBA); // Soft reset
  Wire.endTransmission();
  delay(20);

  // ---- Check ENS160 PART_ID ----
  uint16_t partID = readENS160Word(ENS160_REG_PART_ID);
  if (partID == 0 || partID == 0xFFFF) {
    Serial.println("ENS160 not detected! Check wiring and address.");
  } else {
    Serial.print("ENS160 PART_ID: 0x");
    Serial.println(partID, HEX);

    // ---- Initialize ENS160 ----
    Wire.beginTransmission(ENS160_ADDRESS);
    Wire.write(0xF4); // APP_START
    if (Wire.endTransmission() != 0) {
      Serial.println("Failed to send APP_START");
    } else {
      // Wait for ENS160 ready (DEVICE_STATUS bit 7)
      if (!waitENS160Ready(2000)) { // 2 sec timeout
        Serial.println("ENS160 did not become ready!");
      } else {
        Serial.println("ENS160 Ready!");

        // Put ENS160 in Standard Gas Sensing Mode
        Wire.beginTransmission(ENS160_ADDRESS);
        Wire.write(ENS160_REG_OPMODE);
        Wire.write(0x02);
        Wire.endTransmission();
        delay(100);
      }
    }
  }
}

void loop() {
  float temperature = 0.0, humidity = 0.0;

  // ----- AHT10 Measurement -----
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(80);

  Wire.requestFrom(AHT10_ADDRESS, 6);
  if (Wire.available() == 6) {
    uint8_t data[6];
    for (int i = 0; i < 6; i++) data[i] = Wire.read();

    uint32_t rawHumidity = ((uint32_t)(data[1] << 12)) | ((uint32_t)(data[2] << 4)) | ((data[3] >> 4) & 0x0F);
    humidity = (rawHumidity * 100.0) / 1048576.0;

    uint32_t rawTemp = ((uint32_t)((data[3] & 0x0F) << 16)) | ((uint32_t)(data[4] << 8)) | data[5];
    temperature = ((rawTemp * 200.0) / 1048576.0) - 50.0;

    Serial.print("AHT10 Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("AHT10 Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" % rH");

    // Feed ENS160 with external T & RH
    writeENS160Env(temperature, humidity);
  }

  // Wait until ENS160 has new data (DEVICE_STATUS bit 0)
  if (!waitENS160NewData(1000)) { // 1 sec timeout
    Serial.println("No new ENS160 data available, skipping this cycle.");
    delay(1000);
    return;
  }

  // ----- ENS160 Registers -----
  readENS160Reg(ENS160_REG_OPMODE, 1, "OPMODE");
  readENS160Reg(ENS160_REG_DEVICE_STATUS, 1, "DEVICE_STATUS");

  uint8_t aqi = readENS160Byte(ENS160_REG_DATA_AQI);
  uint16_t tvoc = readENS160Word(ENS160_REG_DATA_TVOC);
  uint16_t eco2 = readENS160Word(ENS160_REG_DATA_ECO2);

  Serial.print("AQI: "); Serial.println(aqi);
  Serial.print("TVOC: "); Serial.print(tvoc); Serial.println(" ppb");
  Serial.print("eCO2: "); Serial.print(eco2); Serial.println(" ppm");

  readENS160Reg(ENS160_REG_DATA_T, 2, "Temperature (used in calc)");
  readENS160Reg(ENS160_REG_DATA_RH, 2, "Humidity (used in calc)");
  readENS160Reg(ENS160_REG_DATA_MISR, 1, "Data Integrity Field");
  readENS160Reg(ENS160_REG_GPR_READ, 8, "GPR Read Registers");

  Serial.println("----------------------");
  delay(1000);
}

// ---------------- Helper Functions ----------------

void writeENS160Env(float temperature, float humidity) {
  int16_t t_raw = (int16_t)((temperature + 273.15) * 64);
  uint16_t rh_raw = (uint16_t)(humidity * 512);

  Wire.beginTransmission(ENS160_ADDRESS);
  Wire.write(ENS160_REG_TEMP_IN);
  Wire.write(t_raw & 0xFF);
  Wire.write((t_raw >> 8) & 0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(ENS160_ADDRESS);
  Wire.write(ENS160_REG_RH_IN);
  Wire.write(rh_raw & 0xFF);
  Wire.write((rh_raw >> 8) & 0xFF);
  Wire.endTransmission();
}

void readENS160Reg(uint8_t reg, uint8_t len, const char* label) {
  Wire.beginTransmission(ENS160_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(ENS160_ADDRESS, len);
  if (Wire.available() == len) {
    Serial.print(label); Serial.print(": ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(Wire.read(), HEX);
      if (i < len - 1) Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.print(label); Serial.println(" read failed");
  }
}

uint8_t readENS160Byte(uint8_t reg) {
  Wire.beginTransmission(ENS160_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ENS160_ADDRESS, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

uint16_t readENS160Word(uint8_t reg) {
  Wire.beginTransmission(ENS160_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ENS160_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    return ((uint16_t)msb << 8) | lsb;
  }
  return 0;
}

// Wait until DEVICE_STATUS bit 7 = 1 (ready)
bool waitENS160Ready(unsigned long timeout_ms) {
  unsigned long start = millis();
  uint8_t status = 0;
  while (millis() - start < timeout_ms) {
    Wire.beginTransmission(ENS160_ADDRESS);
    Wire.write(ENS160_REG_DEVICE_STATUS);
    Wire.endTransmission();
    Wire.requestFrom(ENS160_ADDRESS, 1);
    if (Wire.available()) {
      status = Wire.read();
      Serial.print("DEVICE_STATUS: 0x");
      Serial.println(status, HEX);
      if (status & 0x80) return true;
    }
    delay(50);
  }
  return false;
}

// Wait until DEVICE_STATUS bit 0 = 1 (new data ready)
bool waitENS160NewData(unsigned long timeout_ms) {
  unsigned long start = millis();
  uint8_t status = 0;
  while (millis() - start < timeout_ms) {
    Wire.beginTransmission(ENS160_ADDRESS);
    Wire.write(ENS160_REG_DEVICE_STATUS);
    Wire.endTransmission();
    Wire.requestFrom(ENS160_ADDRESS, 1);
    if (Wire.available()) {
      status = Wire.read();
      if (status & 0x01) return true;
    }
    delay(50);
  }
  return false;
}
