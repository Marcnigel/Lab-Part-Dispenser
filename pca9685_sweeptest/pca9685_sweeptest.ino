#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 on I2C (SDA=pin20, SCL=pin21 on Arduino Mega)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// MG90S-style pulse range (tune if needed)
#define SERVO_MIN   150   // ~0 degrees   (~500us at 50Hz)
#define SERVO_MAX   600   // ~180 degrees (~2500us at 50Hz)

#define FIRST_CH    0
#define LAST_CH     5

#define SDA_PIN     20    // Mega SDA
#define SCL_PIN     21    // Mega SCL

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void moveServo(uint8_t ch, int angle) {
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

// Clear a stuck I2C bus by clocking out a phantom byte, then issue a STOP.
void i2cRecover() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);

  // If SDA is stuck low, a slave is mid-byte -- clock it free
  if (digitalRead(SDA_PIN) == LOW) {
    pinMode(SCL_PIN, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(SCL_PIN, LOW);
      delayMicroseconds(5);
      digitalWrite(SCL_PIN, HIGH);
      delayMicroseconds(5);
      if (digitalRead(SDA_PIN) == HIGH) break;
    }
    // Generate a STOP condition (SDA low->high while SCL high)
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(5);
  }

  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}

void setup() {
  Serial.begin(9600);
  delay(100);                 // let PCA9685 finish its own power-on reset

  i2cRecover();               // clear any stuck bus before starting I2C

  Wire.begin();               // uses pin20 (SDA), pin21 (SCL) on Mega
  Wire.setClock(100000);      // 100kHz -- more forgiving than 400kHz
  pwm.begin();
  pwm.setPWMFreq(50);         // 50Hz for analog servos
  delay(10);

  // Start all at 0
  for (uint8_t ch = FIRST_CH; ch <= LAST_CH; ch++) {
    moveServo(ch, 0);
  }
  delay(500);

  Serial.println("PCA9685 servo control ready.");
  Serial.println("Enter: <channel> <angle>   e.g.  1 0   or   3 90");
}

void loop() {
  if (Serial.available() > 0) {
    int ch = Serial.parseInt();       // first number = channel
    int angle = Serial.parseInt();    // second number = angle

    // clear any leftover characters (newline, etc.)
    while (Serial.available() > 0) Serial.read();

    if (ch < FIRST_CH || ch > LAST_CH) {
      Serial.print("Invalid channel: ");
      Serial.print(ch);
      Serial.println("  (use 0-5)");
      return;
    }
    if (angle < 0 || angle > 180) {
      Serial.print("Invalid angle: ");
      Serial.print(angle);
      Serial.println("  (use 0-180)");
      return;
    }

    moveServo(ch, angle);
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(" -> ");
    Serial.print(angle);
    Serial.println(" deg");
  }
}