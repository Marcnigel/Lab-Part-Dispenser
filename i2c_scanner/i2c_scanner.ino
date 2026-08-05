#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner");
}

void loop() {
  byte count = 0;
  Serial.println("Scanning...");

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }

  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s).\n");
  delay(5000);
}