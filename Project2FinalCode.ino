/*******************************************************
 * Teensy 4.1 — IMU + MAG + GPS + SD + Bluetooth + LED FIX INDICATOR
 * - LSM6DSOX via raw I2C: ACC [m/s^2], GYR [rad/s]
 * - LIS3MDL via Adafruit lib: MAG [µT]
 * - PA1010D GPS via I2C using Adafruit_GPS
 * - HC-06 via UART (Serial1, pins 0/1): CSV stream for GUI
 * - SD card (Teensy 4.1 BUILTIN_SDCARD): /Project2.csv log
 * - LED (LED_BUILTIN): BLINK = searching, SOLID = GPS fix
 *
 * CSV FORMAT (one line per sample):
 * Timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats
 *******************************************************/
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <string.h>
#include <stdlib.h>

// ---------- LED pin ----------
const int LED_PIN = LED_BUILTIN;   // Teensy 4.1 on-board LED (pin 13)
unsigned long ledTimer = 0;
bool ledState = false;

// ---------- I2C addresses ----------
const uint8_t LSM6_ADDR = 0x6A;   // LSM6DSOX
const uint8_t LIS3_ADDR = 0x1C;   // LIS3MDL

// ---------- LSM6DSOX registers ----------
const uint8_t LSM6_WHOAMI   = 0x0F; // expect 0x6C
const uint8_t LSM6_CTRL1_XL = 0x10; // accel cfg
const uint8_t LSM6_CTRL2_G  = 0x11; // gyro cfg
const uint8_t LSM6_CTRL3_C  = 0x12; // BDU/IF_INC
const uint8_t LSM6_OUTX_L_G = 0x22; // GxGyGz 6 bytes
const uint8_t LSM6_OUTX_L_A = 0x28; // AxAyAz 6 bytes

// ---------- chosen scales ----------
/* Gyro: 500 dps -> 17.50 mdps/LSB = 0.0175 dps/LSB -> rad/s */
const float GYRO_DPS_PER_LSB = 0.0175f;
const float DEG2RAD = 3.14159265358979f / 180.0f;

/* Accel: ±4 g -> 0.122 mg/LSB = 0.000122 g/LSB -> m/s^2 */
const float ACC_G_PER_LSB = 0.000122f;
const float G_SI = 9.80665f;

// ---------- Bluetooth HC-06 ----------
const uint32_t HC06_BAUD = 9600;  // HC-06 default, on Serial1

// ---------- MAG (Adafruit LIS3MDL) ----------
Adafruit_LIS3MDL lis3;
bool mag_ok = false;

// ---------- GPS (PA1010D over I2C with Adafruit_GPS) ----------
Adafruit_GPS GPS(&Wire);
const uint8_t GPS_I2C_ADDR = 0x10; // PA1010D default I2C address

// ---------- SD card (Teensy 4.1 built-in slot) ----------
File dataFile;
const int SD_CS = BUILTIN_SDCARD;
const char *LOG_FILENAME = "/Project2.csv";  // match your friend's code

// ---------- tiny I2C helpers for LSM6 ----------
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

// ---------- LSM6DSOX init ----------
bool initLSM6() {
  uint8_t who = 0;
  if (!i2cRead8(LSM6_ADDR, LSM6_WHOAMI, who)) return false;
  if (who != 0x6C) return false;

  // CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) → 0x44
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL3_C, 0x44)) return false;

  // CTRL2_G: ODR_G=104 Hz, FS_G=500 dps → 0x50
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL2_G, 0x50)) return false;

  // CTRL1_XL: ODR_XL=104 Hz, FS_XL=±4 g → 0x50
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL1_XL, 0x50)) return false;

  return true;
}

bool lsm6_ok = false;
uint32_t t_last = 0;

void setup() {
  Serial.begin(115200);  // USB serial to PC (GUI)
  Wire.begin();
  delay(300);

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // HC-06 UART on pins 0 (RX1) and 1 (TX1)
  Serial1.begin(HC06_BAUD);

  // IMU (LSM6DSOX) init
  lsm6_ok = initLSM6();
  if (!lsm6_ok) {
    Serial.println("LSM6DSOX init FAILED");
  } else {
    Serial.println("LSM6DSOX init OK");
  }

  // MAG (LIS3MDL) init
  if (lis3.begin_I2C(LIS3_ADDR)) {
    lis3.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag_ok = true;
    Serial.println("LIS3MDL init OK");
  } else {
    Serial.println("LIS3MDL init FAILED");
  }

  // GPS over I2C
  GPS.begin(GPS_I2C_ADDR);                    // PA1010D I2C address
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // RMC + GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);                // antenna status (optional)
  delay(1000);
  while (GPS.read()) { }  // flush any initial NMEA

  // SD card init (friend-style)
  if (!SD.begin(SD_CS)) {
    Serial.println("Failed to initialize SD card!");
  } else {
    Serial.println("SD card up and running");

    dataFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (dataFile) {
      if (dataFile.size() == 0) {
        // Header matches our actual order:
        // Timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats
        dataFile.println("Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Latitude,Longitude,Altitude_MSL,Satellites");
        dataFile.flush();
      }
      Serial.print("Logging to ");
      Serial.println(LOG_FILENAME);
    } else {
      Serial.println("File failed to open for writing");
    }
  }

  Serial.println("READY");
}

void loop() {
  // ---------- GPS pump ----------
  GPS.read();  // For I2C wrapper, this pulls NMEA sentences
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // ---------- LED Indicator Behavior ----------
  // Base behavior: blink at 1 Hz (so you know loop() is running)
  if (millis() - ledTimer >= 500) {    // toggle every 0.5 s
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // If GPS has a fix, keep LED solid ON
  if (GPS.fix) {
    digitalWrite(LED_PIN, HIGH);
  }

  // ---------- Periodic sample (10 Hz here) ----------
  if (millis() - t_last >= 100) {
    t_last = millis();

    // Time
    unsigned long t_ms = millis();
    unsigned long t_s  = t_ms / 1000;   // nicer for human reading

    float ax_ms2 = 0, ay_ms2 = 0, az_ms2 = 0;
    float gx_rs  = 0, gy_rs  = 0, gz_rs  = 0;
    float mx_uT  = 0, my_uT  = 0, mz_uT  = 0;
    double lat   = 0, lon    = 0, alt   = 0;
    int sats     = 0;

    // --- IMU: GYRO + ACC from LSM6 ---
    if (lsm6_ok) {
      uint8_t b[6];

      // Gyro
      if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_G, b, 6)) {
        int16_t gx_raw = (int16_t)(b[1] << 8 | b[0]);
        int16_t gy_raw = (int16_t)(b[3] << 8 | b[2]);
        int16_t gz_raw = (int16_t)(b[5] << 8 | b[4]);
        gx_rs = gx_raw * (GYRO_DPS_PER_LSB * DEG2RAD);
        gy_rs = gy_raw * (GYRO_DPS_PER_LSB * DEG2RAD);
        gz_rs = gz_raw * (GYRO_DPS_PER_LSB * DEG2RAD);
      }

      // Accel
      if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_A, b, 6)) {
        int16_t ax_raw = (int16_t)(b[1] << 8 | b[0]);
        int16_t ay_raw = (int16_t)(b[3] << 8 | b[2]);
        int16_t az_raw = (int16_t)(b[5] << 8 | b[4]);
        ax_ms2 = ax_raw * (ACC_G_PER_LSB * G_SI);
        ay_ms2 = ay_raw * (ACC_G_PER_LSB * G_SI);
        az_ms2 = az_raw * (ACC_G_PER_LSB * G_SI);
      }
    }

    // --- MAG from LIS3MDL ---
    if (mag_ok) {
      sensors_event_t mag;
      lis3.getEvent(&mag);
      mx_uT = mag.magnetic.x;
      my_uT = mag.magnetic.y;
      mz_uT = mag.magnetic.z;
    }

    // --- GPS from Adafruit_GPS ---
    if (GPS.fix) {
      lat  = GPS.latitudeDegrees;
      lon  = GPS.longitudeDegrees;
      alt  = GPS.altitude;      // meters above MSL
      sats = GPS.satellites;
    }

    // --------- BUILD ONE CSV LINE ----------
    char line[256];
    snprintf(
      line, sizeof(line),
      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%d",
      (unsigned long)t_s,       // Timestamp (s)
      ax_ms2, ay_ms2, az_ms2,
      gx_rs,  gy_rs,  gz_rs,
      mx_uT,  my_uT,  mz_uT,
      lat, lon, alt, sats
    );

    // --------- 1) USB: Serial to GUI ----------
    Serial.println(line);

    // --------- 2) Bluetooth: HC-06 on Serial1 ----------
    Serial1.println(line);

    // --------- 3) SD card logging (/Project2.csv) ----------
    if (dataFile) {
      dataFile.println(line);
      dataFile.flush(); 
    }
  }
}
