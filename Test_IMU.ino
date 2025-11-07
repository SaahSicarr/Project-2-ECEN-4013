/*******************************************************
 * IMU_Test.ino
 * Teensy 4.1 — Test IMU + Magnetometer only
 * - LSM6DSOX via raw I2C: ACC [m/s^2], GYR [rad/s]
 * - LIS3MDL via Adafruit lib: MAG [µT]
 *******************************************************/
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// ---------- I2C addresses ----------
const uint8_t LSM6_ADDR = 0x6A;   // LSM6DSOX / LSM6DS3TR-C
const uint8_t LIS3_ADDR = 0x1C;   // LIS3MDL

// ---------- LSM6DSOX registers ----------
const uint8_t LSM6_WHOAMI   = 0x0F; // expect 0x6C
const uint8_t LSM6_CTRL1_XL = 0x10; // accel cfg
const uint8_t LSM6_CTRL2_G  = 0x11; // gyro cfg
const uint8_t LSM6_CTRL3_C  = 0x12; // BDU/IF_INC
const uint8_t LSM6_OUTX_L_G = 0x22; // GxGyGz
const uint8_t LSM6_OUTX_L_A = 0x28; // AxAyAz

// ---------- chosen scales ----------
const float GYRO_DPS_PER_LSB = 0.0175f;      // 500 dps
const float DEG2RAD = 3.14159265358979f / 180.0f;
const float ACC_G_PER_LSB = 0.000122f;       // ±4 g
const float G_SI = 9.80665f;

// ---------- MAG (Adafruit LIS3MDL) ----------
Adafruit_LIS3MDL lis3;
bool mag_ok = false;

// ---------- tiny I2C helpers ----------
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cReadBuff(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)addr, (int)n);
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    dst[i] = Wire.read();
  }
  return true;
}

bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
  return i2cReadBuff(addr, reg, &val, 1);
}

// ---------- LSM6 init ----------
bool initLSM6() {
  uint8_t who = 0;
  if (!i2cRead8(LSM6_ADDR, LSM6_WHOAMI, who)) return false;
  if (who != 0x6C) return false;

  // CTRL3_C: BDU=1, IF_INC=1
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL3_C, 0x44)) return false;

  // CTRL2_G: 104 Hz, 500 dps
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL2_G, 0x50)) return false;

  // CTRL1_XL: 104 Hz, ±4 g
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL1_XL, 0x50)) return false;

  return true;
}

bool lsm6_ok = false;
uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  Wire.begin();
  delay(200);

  Serial.println("IMU Test: LSM6 + LIS3MDL");

  // Init LSM6
  lsm6_ok = initLSM6();
  if (!lsm6_ok) {
    Serial.println("ERROR: LSM6DS not found at 0x6A. Check wiring.");
  } else {
    Serial.println("LSM6DS initialized.");
  }

  // Init LIS3MDL
  if (lis3.begin_I2C(LIS3_ADDR)) {
    lis3.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag_ok = true;
    Serial.println("LIS3MDL initialized.");
  } else {
    Serial.println("ERROR: LIS3MDL not found at 0x1C. Check wiring.");
  }

  Serial.println("IMU test ready.");
}

void loop() {
  if (millis() - lastPrint < 500) return;
  lastPrint = millis();

  Serial.println("================================");

  // ----- Gyro + Accel -----
  if (lsm6_ok) {
    uint8_t b[6];

    // Gyro
    if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_G, b, 6)) {
      int16_t gx = (int16_t)(b[1] << 8 | b[0]);
      int16_t gy = (int16_t)(b[3] << 8 | b[2]);
      int16_t gz = (int16_t)(b[5] << 8 | b[4]);
      float gx_rs = gx * (GYRO_DPS_PER_LSB * DEG2RAD);
      float gy_rs = gy * (GYRO_DPS_PER_LSB * DEG2RAD);
      float gz_rs = gz * (GYRO_DPS_PER_LSB * DEG2RAD);

      Serial.print("Angular velocity (rad/s): ");
      Serial.print(gx_rs, 3); Serial.print(", ");
      Serial.print(gy_rs, 3); Serial.print(", ");
      Serial.println(gz_rs, 3);
    } else {
      Serial.println("Angular velocity (rad/s): -, -, -");
    }

    // Accel
    if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_A, b, 6)) {
      int16_t ax = (int16_t)(b[1] << 8 | b[0]);
      int16_t ay = (int16_t)(b[3] << 8 | b[2]);
      int16_t az = (int16_t)(b[5] << 8 | b[4]);
      float ax_ms2 = ax * (ACC_G_PER_LSB * G_SI);
      float ay_ms2 = ay * (ACC_G_PER_LSB * G_SI);
      float az_ms2 = az * (ACC_G_PER_LSB * G_SI);

      Serial.print("Acceleration (m/s^2): ");
      Serial.print(ax_ms2, 3); Serial.print(", ");
      Serial.print(ay_ms2, 3); Serial.print(", ");
      Serial.println(az_ms2, 3);
    } else {
      Serial.println("Acceleration (m/s^2): -, -, -");
    }
  } else {
    Serial.println("Angular velocity (rad/s): -, -, -");
    Serial.println("Acceleration (m/s^2): -, -, -");
  }

  // ----- Magnetometer -----
  if (mag_ok) {
    sensors_event_t mag;
    lis3.getEvent(&mag);
    Serial.print("Magnetic field (uT): ");
    Serial.print(mag.magnetic.x, 1); Serial.print(", ");
    Serial.print(mag.magnetic.y, 1); Serial.print(", ");
    Serial.println(mag.magnetic.z, 1);
  } else {
    Serial.println("Magnetic field (uT): -, -, -");
  }

  Serial.println();
}
