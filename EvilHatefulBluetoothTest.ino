// Teensy 4.1 + HC-05 normal data bridge on Serial1 (pins 0/1)

void setup() {
  Serial.begin(9600);          // USB Serial (Teensy -> COM9)
  while (!Serial && millis() < 4000) {}

  Serial1.begin(9600);         // HC-05 default data baud
  Serial.println("HC-05 data bridge running.");
}

void loop() {
  // Bluetooth -> USB
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }

  // USB -> Bluetooth
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
  }
}
