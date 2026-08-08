#include <Servo.h>

Servo bigServo;

void setup() {
  Serial.begin(9600);
  bigServo.attach(16, 500, 2500);
  //bigServo.write(0);
  Serial.println("Enter angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    if (angle >= 0 && angle <= 180) {
      bigServo.write(angle);
      Serial.print("Moving to ");
      Serial.print(angle);
      Serial.println("°");
    } else {
      Serial.println("Out of range (0-180)");
    }

    // flush leftover characters (newline, etc.)
    while (Serial.available() > 0) Serial.read();
  }
}