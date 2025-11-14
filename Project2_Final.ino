/*******************************************************
 * Project2_Final.ino
 * Teensy 4.1 — Integrated system for Project 2
 * - LSM6DSOX + LIS3MDL (I2C)
 * - GPS (UART1)
 * - Bluetooth (UART2, optional)
 * - SD card logging (built-in slot)
 *******************************************************/
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

// ---------- Addresses ----------
#define LSM6_ADDR 0x6A
#define LIS3_ADDR 0x1C
#define GPS_BAUD 9600

// ---------- Pins ----------
#define BT_RX 7
#define BT_TX 8

// ---------- IMU registers ----------
#define LSM6_WHOAMI   0x0F
#define LSM6_CTRL1_XL 0x10
#define LSM6_CTRL2_G  0x11
#define LSM6_CTRL3_C  0x12
#define LSM6_OUTX_L_G 0x22
#define LSM6_OUTX_L_A 0x28

// ---------- Scaling ----------
const float GYRO_DPS_PER_LSB = 0.0175f;
const float DEG2RAD = 3.14159265358979f / 180.0f;
const float ACC_G_PER_LSB = 0.000122f;
const float G_SI = 9.80665f;

// ---------- GPS data struct ----------
struct GPSFix {
  bool valid = false;
  double lat = 0, lon = 0;
  double alt_msl = 0, sep = 0;
  bool have_msl = false, have_sep = false;
  int sats = 0;
} gps;

char nmea[128];
uint8_t nmea_len = 0;

// ---------- Objects ----------
Adafruit_LIS3MDL lis3;
bool mag_ok = false;
bool lsm6_ok = false;

// ---------- SD ----------
File logFile;

// ---------- Bluetooth ----------
bool bt_ok = false;

// ---------- I2C helpers ----------
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

// ---------- GPS helpers ----------
static double parseCoordDeg(const char* field, bool isLat) {
  if (!field || !*field) return 0.0;
  double v = atof(field);
  int deg = (int)(v / 100.0);
  double minutes = v - (deg * 100.0);
  return deg + minutes / 60.0;
}
static uint8_t hex2u8(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return 0;
}
static bool nmeaChecksumOK(const char* s) {
  if (*s != '$') return false;
  uint8_t cs = 0;
  s++;
  while (*s && *s != '*') cs ^= (uint8_t)*s++;
  if (*s != '*') return false;
  s++;
  uint8_t given = (hex2u8(s[0]) << 4) | hex2u8(s[1]);
  return cs == given;
}
static void parseGGA(char* line) {
  int f = 0; char* tok = strtok(line, ",");
  while (tok) {
    switch (f) {
      case 2: gps.lat = parseCoordDeg(tok, true); break;
      case 3: if (*tok == 'S') gps.lat = -gps.lat; break;
      case 4: gps.lon = parseCoordDeg(tok, false); break;
      case 5: if (*tok == 'W') gps.lon = -gps.lon; break;
      case 6: gps.valid = (*tok != '0'); break;
      case 7: gps.sats = atoi(tok); break;
      case 9: gps.alt_msl = atof(tok); gps.have_msl = true; break;
      case 11: gps.sep = atof(tok); gps.have_sep = true; break;
    }
    tok = strtok(NULL, ",");
    f++;
  }
}
static void parseRMC(char* line) {
  int f = 0; char* tok = strtok(line, ",");
  bool ok = false; double lat = 0, lon = 0; char ns = 0, ew = 0;
  while (tok) {
    switch (f) {
      case 2: ok = (*tok == 'A'); break;
      case 3: lat = parseCoordDeg(tok, true); break;
      case 4: ns = *tok; break;
      case 5: lon = parseCoordDeg(tok, false); break;
      case 6: ew = *tok; break;
    }
    tok = strtok(NULL, ","); f++;
  }
  if (ok) {
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;
    gps.lat = lat; gps.lon = lon; gps.valid = true;
  }
}
static void handleNMEA() {
  if (nmea_len < 6 || nmea[0] != '$' || !nmeaChecksumOK(nmea)) return;
  char buf[128]; strncpy(buf, nmea, sizeof(buf)); buf[127] = 0;
  if (strstr(buf, "GGA")) parseGGA(buf);
  else if (strstr(buf, "RMC")) parseRMC(buf);
}

// ---------- LSM6 init ----------
bool initLSM6() {
  uint8_t who = 0;
  if (!i2cRead8(LSM6_ADDR, LSM6_WHOAMI, who)) return false;
  if (who != 0x6C) return false;
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL3_C, 0x44)) return false;
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL2_G, 0x50)) return false;
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL1_XL, 0x50)) return false;
  return true;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();
  delay(200);

  Serial1.begin(GPS_BAUD);
  Serial2.begin(9600);  // Bluetooth

  // IMU setup
  lsm6_ok = initLSM6();
  mag_ok = lis3.begin_I2C(LIS3_ADDR);
  if (mag_ok) {
    lis3.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  }

  // SD setup
  if (SD.begin(BUILTIN_SDCARD)) {
    logFile = SD.open("log.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats");
      logFile.flush();
    }
  }

  Serial.println("System ready.");
}

// ---------- Loop ----------
uint32_t lastPrint = 0;
void loop() {
  // --- Read GPS stream ---
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') { nmea[nmea_len] = 0; handleNMEA(); nmea_len = 0; }
    else if (nmea_len < sizeof(nmea) - 1) nmea[nmea_len++] = c;
  }

  // --- Print/log every 500 ms ---
  if (millis() - lastPrint < 500) return;
  lastPrint = millis();

  uint8_t b[6];
  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  sensors_event_t mag;

  if (lsm6_ok) {
    if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_A, b, 6)) {
      ax = (int16_t)(b[1]<<8|b[0]) * (ACC_G_PER_LSB * G_SI);
      ay = (int16_t)(b[3]<<8|b[2]) * (ACC_G_PER_LSB * G_SI);
      az = (int16_t)(b[5]<<8|b[4]) * (ACC_G_PER_LSB * G_SI);
    }
    if (i2cReadBuff(LSM6_ADDR, LSM6_OUTX_L_G, b, 6)) {
      gx = (int16_t)(b[1]<<8|b[0]) * (GYRO_DPS_PER_LSB * DEG2RAD);
      gy = (int16_t)(b[3]<<8|b[2]) * (GYRO_DPS_PER_LSB * DEG2RAD);
      gz = (int16_t)(b[5]<<8|b[4]) * (GYRO_DPS_PER_LSB * DEG2RAD);
    }
  }

  if (mag_ok) lis3.getEvent(&mag);

  double altEllip = (gps.have_msl && gps.have_sep) ? gps.alt_msl + gps.sep : NAN;

  // --- Build one CSV line ---
  String line = String(millis()) + "," +
                String(ax,3)+","+String(ay,3)+","+String(az,3)+","+
                String(gx,3)+","+String(gy,3)+","+String(gz,3)+","+
                String(mag.magnetic.x,1)+","+String(mag.magnetic.y,1)+","+String(mag.magnetic.z,1)+","+
                String(gps.lat,6)+","+String(gps.lon,6)+","+
                String(altEllip,1)+","+String(gps.sats);

  Serial.println(line);
  Serial2.println(line);  // Bluetooth
  if (logFile) { logFile.println(line); logFile.flush(); }
}
