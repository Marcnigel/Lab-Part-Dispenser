// Mega 2560 pin tester
// Drives every digital pin HIGH/LOW so you can verify with a DMM

const int TOTAL_PINS = 70;   // D0..D69 (D54..D69 = A0..A15)
int mode = 1;                // 1 = all HIGH, 0 = all LOW

void setup() {
  for (int p = 0; p < TOTAL_PINS; p++) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }
  Serial.begin(9600);   // NOTE: D0/D1 are the USB serial pins
}

void loop() {
  mode = !mode;
  for (int p = 0; p < TOTAL_PINS; p++) {
    digitalWrite(p, mode ? HIGH : LOW);
  }
  Serial.println(mode ? "ALL PINS HIGH" : "ALL PINS LOW");
  delay(2000);   // 2s HIGH, 2s LOW — slow enough to probe by hand
}