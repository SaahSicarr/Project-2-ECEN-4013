/*******************************************************
 * SD_Test.ino
 * Teensy 4.1 — Test built-in microSD card
 *
 * Functions:
 *  - Initialize SD card using BUILTIN_SDCARD
 *  - List all files in root directory
 *  - Read and print "data.txt" if it exists
 *******************************************************/

#include <Arduino.h>
#include <SD.h>

void listDirectory(File dir, int numTabs);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for Serial Monitor
  }

  Serial.println("Teensy 4.1 microSD Test");

  // Initialize SD using internal SDIO interface
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("ERROR: SD init failed. Check card is inserted and formatted (FAT32).");
    while (1) {
      // stop here if SD fails
    }
  }

  Serial.println("SD init successful.");

  // List all files in root directory
  Serial.println("\nRoot directory contents:");
  File root = SD.open("/");
  if (!root) {
    Serial.println("ERROR: Cannot open root directory.");
  } else {
    listDirectory(root, 0);
    root.close();
  }

  // Try to open a specific file (edit name as needed)
  const char *filename = "data.txt";
  Serial.print("\nReading file: ");
  Serial.println(filename);

  if (SD.exists(filename)) {
    File f = SD.open(filename, FILE_READ);
    if (f) {
      Serial.println("----- File start -----");
      while (f.available()) {
        Serial.write(f.read());
      }
      Serial.println("\n----- File end -----");
      f.close();
    } else {
      Serial.println("ERROR: Could not open file (permissions / corruption?).");
    }
  } else {
    Serial.println("File does not exist on SD card.");
  }

  Serial.println("\nSD test complete.");
}

void loop() {
  // Nothing here: one-shot test on boot
}

// Recursively list files and folders
void listDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      break;
    }

    for (int i = 0; i < numTabs; i++) {
      Serial.print("  ");
    }

    Serial.print(entry.name());

    if (entry.isDirectory()) {
      Serial.println("/");
      listDirectory(entry, numTabs + 1);
    } else {
      Serial.print("  (");
      Serial.print(entry.size());
      Serial.println(" bytes)");
    }

    entry.close();
  }
}
