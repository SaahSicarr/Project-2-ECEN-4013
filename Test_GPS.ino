/*******************************************************
 * GPS_Test.ino
 * Teensy 4.1 — Test GPS only (PA1010D via UART / Serial1)
 * Outputs:
 * - Latitude (deg)
 * - Longitude (deg)
 * - Elevation (m, MSL)
 * - Elevation (m, Ellipsoid)
 * - Number of locked satellites
 *******************************************************/
#include <Arduino.h>

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

// ---------- Helpers ----------
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
  while (*s && *s != '*') {
    cs ^= (uint8_t)*s++;
  }
  if (*s != '*') return false;
  s++;
  uint8_t given = (hex2u8(s[0]) << 4) | hex2u8(s[1]);
  return cs == given;
}

static void parseGGA(char* line) {
  int f = 0;
  char* tok = strtok(line, ",");
  while (tok) {
    switch (f) {
      case 2: gps.lat_deg = parseCoordDeg(tok, true); break;
      case 3: if (*tok == 'S') gps.lat_deg = -gps.lat_deg; break;
      case 4: gps.lon_deg = parseCoordDeg(tok, false); break;
      case 5: if (*tok == 'W') gps.lon_deg = -gps.lon_deg; break;
      case 6: gps.valid = (*tok != '0'); break;
      case 7: gps.sats = atoi(tok); break;
      case 9: gps.alt_msl_m = atof(tok); gps.have_msl = true; break;
      case 11: gps.geoid_sep_m = atof(tok); gps.have_sep = true; break;
    }
    tok = strtok(NULL, ",");
    f++;
  }
}

static void parseRMC(char* line) {
  int f = 0;
  char* tok = strtok(line, ",");
  bool ok = false;
  double lat = 0, lon = 0;
  char ns = 0, ew = 0;

  while (tok) {
    switch (f) {
      case 2: ok = (*tok == 'A'); break;
      case 3: lat = parseCoordDeg(tok, true); break;
      case 4: ns = *tok; break;
      case 5: lon = parseCoordDeg(tok, false); break;
      case 6: ew = *tok; break;
    }
    tok = strtok(NULL, ",");
    f++;
  }

  if (ok) {
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;
    gps.lat_deg = lat;
    gps.lon_deg = lon;
    gps.valid = true;
  }
}

static void handleNMEA() {
  if (nmea_len < 6 || nmea[0] != '$' || !nmeaChecksumOK(nmea)) return;

  char buf[128];
  strncpy(buf, nmea, sizeof(buf));
  buf[127] = 0;

  if (strstr(buf, "GGA")) {
    parseGGA(buf);
  } else if (strstr(buf, "RMC")) {
    parseRMC(buf);
  }
}

uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  Serial1.begin(GPS_BAUD);

  Serial.println("GPS Test: PA1010D on Serial1");
  Serial.println("Go outside or near a window for a fix.");
}

void loop() {
  // Read NMEA from GPS
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
      nmea_len = 0;
    }
  }

  // Print once per second
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  Serial.println("================================");

  if (gps.valid) {
    double altEllip = (gps.have_msl && gps.have_sep)
                      ? (gps.alt_msl_m + gps.geoid_sep_m)
                      : NAN;

    Serial.print("Latitude (deg): ");
    Serial.println(gps.lat_deg, 6);

    Serial.print("Longitude (deg): ");
    Serial.println(gps.lon_deg, 6);

    Serial.print("Elevation (m, MSL): ");
    if (gps.have_msl) Serial.println(gps.alt_msl_m, 2);
    else Serial.println("-");

    Serial.print("Elevation (m, Ellipsoid): ");
    if (!isnan(altEllip)) Serial.println(altEllip, 2);
    else Serial.println("-");

    Serial.print("Locked satellites: ");
    Serial.println(gps.sats);
  } else {
    Serial.println("GPS: acquiring fix... (no valid data yet)");
  }

  Serial.println();
}
