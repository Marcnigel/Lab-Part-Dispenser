const uint8_t SW_PIN = 0;
int lastState = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(SW_PIN, INPUT_PULLUP);   // switch wired to GND
  lastState = digitalRead(SW_PIN);
}

void loop() {
  int state = digitalRead(SW_PIN);
  if (state != lastState) {
    delay(20);                     // debounce
    state = digitalRead(SW_PIN);
    //if (state != lastState) {
      Serial.print("Pin 17 changed to: ");
      Serial.println(state == LOW ? "PRESSED (LOW)" : "RELEASED (HIGH)");
      lastState = state;
    //}
  }
}