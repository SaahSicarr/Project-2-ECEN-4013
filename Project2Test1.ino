// I2C Scanner: lists device addresses found on SDA(18)/SCL(19)
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();                    // Teensy default I2C pins 18/19
  delay(500);
  Serial.println("I2C scan...");
}
void loop() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found: 0x"); Serial.println(addr, HEX);
      count++;
      delay(2);
    }
  }
  Serial.print("Total devices: "); Serial.println(count);
  Serial.println("----");
  delay(2000);
}
