#include <Servo.h>

Servo bigServo;

void setup() {
  bigServo.attach(14, 500, 2500);
}

void loop() {
  bigServo.writeMicroseconds(500);   // 0°
  delay(2000);
  bigServo.writeMicroseconds(2089);  // ~170°
  delay(2000);
}