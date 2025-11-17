/*******************************************************
 * Teensy 4.1 — IMU+MAG+GPS demo without LSM6DS library
 * - LSM6DSOX via raw I2C: ACC [m/s^2], GYR [rad/s]
 * - LIS3MDL via Adafruit lib: MAG [µT]
 * - GPS via UART (Serial1): lat, lon, alt (MSL), sats
 * 
 * OUTPUT FORMAT (one line per sample, CSV):
 * time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats
 *******************************************************/
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <stdlib.h>

// ---------- I2C addresses ----------
const uint8_t LSM6_ADDR = 0x6A;   // from your scan
const uint8_t LIS3_ADDR = 0x1C;   // from your scan

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

// ---------- GPS ----------
const uint32_t GPS_BAUD = 9600;
struct GPSFix {
  bool   valid = false;
  double lat_deg = 0, lon_deg = 0;
  double alt_msl_m = 0, geoid_sep_m = 0;
  bool   have_msl = false, have_sep = false;
  int    sats = 0;
} gps;
char nmea[128]; 
uint8_t nmea_len = 0;

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
  for (uint8_t i=0; i<n; i++) {
    if (!Wire.available()) return false;
    dst[i] = Wire.read();
  }
  return true;
}
bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
  if (!i2cReadBuff(addr, reg, &val, 1)) return false;
  return true;
}

// ---------- NMEA helpers ----------
static double parseCoordDeg(const char* field, bool isLat) {
  if (!field || !*field) return 0.0;
  double v = atof(field);
  int deg = (int)(v / 100.0);
  double minutes = v - (deg * 100.0);
  return deg + minutes / 60.0;
}
static uint8_t hex2u8(char c){
  if (c>='0'&&c<='9') return c-'0';
  if (c>='A'&&c<='F') return 10+(c-'A');
  if (c>='a'&&c<='f') return 10+(c-'a');
  return 0;
}
static bool nmeaChecksumOK(const char* s){
  if (*s!='$') return false;
  uint8_t cs=0; s++;
  while (*s && *s!='*') cs ^= (uint8_t)*s++;
  if (*s!='*') return false; 
  s++;
  return cs == ((hex2u8(s[0])<<4)|hex2u8(s[1]));
}
static void parseGGA(char* line){
  int f=0; 
  char* tok=strtok(line,",");
  while(tok){
    switch(f){
      case 2: gps.lat_deg=parseCoordDeg(tok,true);break;
      case 3: if(*tok=='S') gps.lat_deg=-gps.lat_deg; break;
      case 4: gps.lon_deg=parseCoordDeg(tok,false);break;
      case 5: if(*tok=='W') gps.lon_deg=-gps.lon_deg; break;
      case 6: gps.valid = (*tok!='0'); break;
      case 7: gps.sats = atoi(tok); break;
      case 9: gps.alt_msl_m = atof(tok); gps.have_msl=true; break;
      case 11:gps.geoid_sep_m = atof(tok); gps.have_sep=true; break;
    }
    tok=strtok(NULL,","); 
    f++;
  }
}
static void parseRMC(char* line){
  int f=0; 
  char* tok=strtok(line,",");
  bool ok=false; 
  double lat=0,lon=0; 
  char ns=0,ew=0;
  while(tok){
    switch(f){
      case 2: ok=(*tok=='A'); break;
      case 3: lat=parseCoordDeg(tok,true); break;
      case 4: ns=*tok; break;
      case 5: lon=parseCoordDeg(tok,false); break;
      case 6: ew=*tok; break;
    }
    tok=strtok(NULL,","); 
    f++;
  }
  if(ok){ 
    if(ns=='S') lat=-lat; 
    if(ew=='W') lon=-lon;
    gps.lat_deg=lat; 
    gps.lon_deg=lon; 
    gps.valid=true; 
  }
}
static void handleNMEA(){
  if (nmea_len<6 || nmea[0]!='$' || !nmeaChecksumOK(nmea)) return;
  char buf[128]; 
  strncpy(buf,nmea,sizeof(buf)); 
  buf[127]=0;
  if (strstr(buf,"GGA"))      parseGGA(buf);
  else if (strstr(buf,"RMC")) parseRMC(buf);
}

// ---------- LSM6DSOX init ----------
bool initLSM6(){
  uint8_t who=0;
  if (!i2cRead8(LSM6_ADDR, LSM6_WHOAMI, who)) return false;
  if (who != 0x6C) return false;

  // CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) → 0b0100_0100 = 0x44
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL3_C, 0x44)) return false;

  // CTRL2_G: ODR_G=104 Hz (0100xxxx), FS_G=500 dps (xx01xxxx) → 0x50
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL2_G, 0x50)) return false;

  // CTRL1_XL: ODR_XL=104 Hz (0100xxxx), FS_XL=±4 g (xx01xxxx) → 0x50
  if (!i2cWrite8(LSM6_ADDR, LSM6_CTRL1_XL, 0x50)) return false;

  return true;
}

bool lsm6_ok = false;
uint32_t t_last = 0;

void setup() {
  Serial.begin(115200);   // USB serial to PC (GUI)
  Wire.begin();
  delay(300);

  // GPS UART
  Serial1.begin(GPS_BAUD);

  // Init LSM6 via raw I2C
  lsm6_ok = initLSM6();
  if (!lsm6_ok) Serial.println("LSM6DSOX init FAILED");

  // Init LIS3MDL (mag)
  if (lis3.begin_I2C(LIS3_ADDR)) {
    lis3.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag_ok = true;
  } else {
    Serial.println("LIS3MDL init FAILED");
  }

  // Optional: one line the GUI will ignore (no commas)
  Serial.println("READY");
}

void loop() {
  // --- GPS read / NMEA accumulate ---
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      nmea[nmea_len] = 0;
      handleNMEA();
      nmea_len = 0;
    } else if (nmea_len < sizeof(nmea) - 1) {
      nmea[nmea_len++] = c;
    } else {
      nmea_len = 0;  // overflow, reset
    }
  }

  // Sample + print about every 100 ms
  if (millis() - t_last >= 100) {
    t_last = millis();

    // --------- READ SENSORS ----------
    // Defaults if something fails
    float ax_ms2 = 0, ay_ms2 = 0, az_ms2 = 0;
    float gx_rs  = 0, gy_rs  = 0, gz_rs  = 0;
    float mx_uT  = 0, my_uT  = 0, mz_uT  = 0;
    double lat   = 0, lon    = 0, alt   = 0;
    int sats     = 0;

    // --- GYRO + ACC from LSM6 ---
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

    // --- GPS ---
    if (gps.valid) {
      lat  = gps.lat_deg;
      lon  = gps.lon_deg;
      sats = gps.sats;

      if (gps.have_msl) {
        alt = gps.alt_msl_m;  // using MSL altitude for GUI
      } else {
        alt = 0;              // or leave as 0 if unknown
      }
    }

    // --------- PRINT ONE CSV LINE ----------
    // Format: time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats

    Serial.print(millis());      Serial.print(',');
    Serial.print(ax_ms2, 3);     Serial.print(',');
    Serial.print(ay_ms2, 3);     Serial.print(',');
    Serial.print(az_ms2, 3);     Serial.print(',');
    Serial.print(gx_rs, 3);      Serial.print(',');
    Serial.print(gy_rs, 3);      Serial.print(',');
    Serial.print(gz_rs, 3);      Serial.print(',');
    Serial.print(mx_uT, 3);      Serial.print(',');
    Serial.print(my_uT, 3);      Serial.print(',');
    Serial.print(mz_uT, 3);      Serial.print(',');
    Serial.print(lat, 6);        Serial.print(',');
    Serial.print(lon, 6);        Serial.print(',');
    Serial.print(alt, 2);        Serial.print(',');
    Serial.print(sats);
    Serial.println();
  }
}
