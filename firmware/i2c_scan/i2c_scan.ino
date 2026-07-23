#include <Wire.h>

#define TFT_BL 19

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Test backlight
  pinMode(TFT_BL, OUTPUT);
  Serial.println("Testing TFT backlight on GPIO19...");
  Serial.println("Watch the display - backlight should blink on/off");

  // I2C scan
  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println("\nI2C Scanner");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("Found %d I2C device(s)\n\n", found);
}

void loop() {
  // Blink backlight - should be visible even without SPI working
  Serial.println("Backlight ON");
  digitalWrite(TFT_BL, HIGH);
  delay(1000);

  Serial.println("Backlight OFF");
  digitalWrite(TFT_BL, LOW);
  delay(1000);
}
