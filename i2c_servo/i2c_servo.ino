#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
 
// ── PCA9685 instance (default I2C address 0x40) ──────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
 
// ── MG90S pulse width limits (microseconds) ──────────────────────────────────
// Tower Pro MG90S: 500 µs = 0°,  2400 µs = 180°
// If a servo doesn't reach its full end stop, nudge these by ±50 µs.
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2400
 
//PWM frequency
#define PWM_FREQ       50
 
//Servo configuration 
#define NUM_SERVOS     4
 
// Map a logical servo index to its PCA9685 channel (0–15)
// Edit these to match your physical wiring
uint8_t servoChannel[NUM_SERVOS] = { 0, 1, 2, 3 };
 
 
// ── Helper: convert an angle (0–180°) to a PCA9685 12-bit tick count ─────────
uint16_t angleToPulse(float degrees) {
  degrees = constrain(degrees, 0.0f, 180.0f);
 
  // Linear map: degrees → pulse width in microseconds
  float pulseUs = SERVO_MIN_US + (degrees / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
 
  // At 50 Hz the period is 20 000 µs, mapped to 4096 ticks by the PCA9685
  return (uint16_t)((pulseUs / 20000.0f) * 4096.0f);
}
 
// ── Move a servo to a target angle ───────────────────────────────────────────
void setServoAngle(uint8_t servoIndex, float degrees) {
  if (servoIndex >= NUM_SERVOS) return;
  pca.setPWM(servoChannel[servoIndex], 0, angleToPulse(degrees));
}
 
// ── Release a servo (stops holding torque — reduces heat on MG90S) ───────────
void releaseServo(uint8_t servoIndex) {
  if (servoIndex >= NUM_SERVOS) return;
  pca.setPWM(servoChannel[servoIndex], 0, 0);
}
 
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Wire.begin();
  pca.begin();
  pca.setOscillatorFrequency(27000000);   // PCA9685 internal oscillator: 27 MHz
  pca.setPWMFreq(PWM_FREQ);               // 50 Hz for MG90S
  delay(10);
 
  Serial.println("Ready.");
}
 
void loop() {

  //independent positioning
  Serial.println("Individual positions...");
  setServoAngle(0, 0);
  delay(1500);
  setServoAngle(0, 180);
  delay(1500);
  
}