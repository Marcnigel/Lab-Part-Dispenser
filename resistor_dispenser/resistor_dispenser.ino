// ================================================================
//  Resistor Dispenser
//  DRV8825 Stepper (via 74HC595 enable) + analog photo-interrupter
//  + PCA9685 dual-servo cutter
//  Serial commands:
//    <number>  -> dispense that many resistors
//    load      -> feed until one resistor is detected at the sensor
//    status    -> print current load status (EMPTY / LOADED / REMOVE)
//  After a successful load, the next dispense feeds one extra resistor.
//  Sensor edge detection via software Schmitt trigger.
//  Cutter: servo 0 (0->90) and servo 1 (90->0) move together, then return.
//  Dispensing is chunked: at most 2 resistors per cut
//  (e.g. a 5-count command -> 2, cut, 2, cut, 1, cut).
// ================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// -- Shift Register 1 --------------------------------------------
#define SR1_SER    5
#define SR1_RCLK   6
#define SR1_SRCLK  7

// -- Motor 1 -----------------------------------------------------
#define M1_STEP    46
#define M1_DIR     47
#define M1_EN_BIT  4

// -- Photo-interrupter (Schmitt trigger) -------------------------
#define SENSOR_PIN   A4
#define SENSOR_HI    900   // must rise above this -> BLOCKED
#define SENSOR_LO    440   // must fall below this -> CLEAR

// -- Dispense feed profile ---------------------------------------
#define FEED_DIR         1
#define FEED_MODE        32
#define FEED_DELAY_US    100
#define STEP_TIMEOUT   30000
#define STALL_STEPS     9000   // no sensor change within this many steps -> stuck

// -- Cut chunking ------------------------------------------------
#define MAX_PER_CUT      2     // dispense at most this many before cutting

// -- PCA9685 cutter servos ---------------------------------------
#define PCA_ADDR      0x40
#define SERVO_MIN     150   // ~0 degrees   (~500us at 50Hz)
#define SERVO_MAX     600   // ~180 degrees (~2500us at 50Hz)

#define CUT_CH_A      2  // servo 0: rest 0 -> cut 90
#define CUT_CH_B      3     // servo 1: rest 90 -> cut 0

#define CUT_A_REST    90
#define CUT_A_CUT     0
#define CUT_B_REST    0
#define CUT_B_CUT     90

#define CUT_TRAVEL_MS 400   // time to reach each end (tune)
#define CUT_HOLD_MS   300   // dwell at the cut before retracting

#define SDA_PIN       20    // Mega SDA
#define SCL_PIN       21    // Mega SCL

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

byte sr1 = 0xFF;

// -- Schmitt-trigger state ---------------------------------------
bool s1_blocked = false;
bool needExtra  = false;   // set true after a successful load

// -- Load status flag --------------------------------------------
enum LoadStatus { EMPTY, LOADED, REMOVE };
LoadStatus loadState = EMPTY;

const char* statusName(LoadStatus s) {
  switch (s) {
    case EMPTY:  return "EMPTY";
    case LOADED: return "LOADED";
    case REMOVE: return "REMOVE";
  }
  return "?";
}


// -- Shift Register Functions ------------------------------------
void shiftOut595(byte ser, byte rclk, byte srclk, byte val) {
  digitalWrite(rclk, LOW);
  shiftOut(ser, srclk, MSBFIRST, val);
  digitalWrite(rclk, HIGH);
}

void setEnableBit(byte ser, byte rclk, byte srclk, byte& state, byte bit, bool value) {
  if (value) state |=  (1u << bit);
  else       state &= ~(1u << bit);
  shiftOut595(ser, rclk, srclk, state);
}

void enableMotor(byte ser, byte rclk, byte srclk, byte& state, byte bit, bool enable) {
  setEnableBit(ser, rclk, srclk, state, bit, !enable);   // active-LOW
}

void disableAllMotors() {
  sr1 = 0xFF;                                  // all enable bits HIGH = disabled (active-LOW)
  shiftOut595(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
}

// -- Low-level Stepper -------------------------------------------
void pulseStep(byte stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

void stepMotor(byte dirPin, byte stepPin, int steps, int mode = 1, int delayUs = 800) {
  digitalWrite(dirPin, steps >= 0 ? HIGH : LOW);
  long count = (long)abs(steps) * mode;
  for (long i = 0; i < count; i++) pulseStep(stepPin, delayUs);
}


// -- Sensor: software Schmitt trigger ----------------------------
bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) {
    if (v < lo) state = false;
  } else {
    if (v > hi) state = true;
  }
  return state;
}


// -- PCA9685 servo helpers ---------------------------------------
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

  if (digitalRead(SDA_PIN) == LOW) {
    pinMode(SCL_PIN, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(SCL_PIN, LOW);
      delayMicroseconds(5);
      digitalWrite(SCL_PIN, HIGH);
      delayMicroseconds(5);
      if (digitalRead(SDA_PIN) == HIGH) break;
    }
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


// -- Cutter ------------------------------------------------------
// One cut: both servos rest -> cut (simultaneously) -> rest.
//   servo 0:  0 -> 90 -> 0
//   servo 1: 90 ->  0 -> 90
void cutReel() {
  moveServo(CUT_CH_A, CUT_A_CUT);
  moveServo(CUT_CH_B, CUT_B_CUT);
  delay(CUT_TRAVEL_MS + CUT_HOLD_MS);

  moveServo(CUT_CH_A, CUT_A_REST);
  moveServo(CUT_CH_B, CUT_B_REST);
  delay(CUT_TRAVEL_MS);
}


// -- Feed one resistor -------------------------------------------
bool feedOneResistor(byte dirPin, byte stepPin, bool& sensorState,
                     int dir, int mode, int delayUs, long timeoutSteps) {
  digitalWrite(dirPin, dir >= 0 ? HIGH : LOW);
  long pulses = 0;

  schmitt(SENSOR_PIN, sensorState, SENSOR_HI, SENSOR_LO);

  while (!schmitt(SENSOR_PIN, sensorState, SENSOR_HI, SENSOR_LO)) {
    pulseStep(stepPin, delayUs);
    if (++pulses > timeoutSteps) return false;
  }
  while (schmitt(SENSOR_PIN, sensorState, SENSOR_HI, SENSOR_LO)) {
    pulseStep(stepPin, delayUs);
    if (++pulses > timeoutSteps) return false;
  }
  return true;
}


// -- Load: feed until a resistor is detected at the sensor -------
// Sets loadState:
//   LOADED  -> object detected at the beam (ready); arms one-extra dispense
//   REMOVE  -> motor stepping but sensor state won't change (strip stuck / empty)
LoadStatus loadResistor() {
  // if already blocked at rest, an object is sitting at the beam -> loaded
  if (schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO)) {
    loadState = LOADED;
    needExtra = true;
    return loadState;
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  delay(5);

  digitalWrite(M1_DIR, FEED_DIR >= 0 ? HIGH : LOW);
  long sinceChange = 0;
  bool prev = s1_blocked;
  loadState = EMPTY;

  while (true) {
    pulseStep(M1_STEP, FEED_DELAY_US);
    bool now = schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO);

    if (now != prev) {                        // strip is moving
      prev = now;
      sinceChange = 0;
      if (now) { loadState = LOADED; break; } // became BLOCKED -> loaded
    } else if (++sinceChange > STALL_STEPS) { // no change for too long
      loadState = REMOVE;                     // stuck / strip too short
      break;
    }
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
  if (loadState == LOADED) needExtra = true;  // arm one-extra dispense
  return loadState;
}


// -- High-level Dispense -----------------------------------------
// Dispenses in chunks of at most MAX_PER_CUT, cutting after each chunk.
// e.g. qty = 5 -> 2 (cut), 2 (cut), 1 (cut).
int dispenseResistors(int qty) {
  if (qty <= 0) return 0;

  int target = qty;
  if (needExtra) {
    target += 2;
    needExtra = false;
    Serial.println("(loaded: dispensing two extra)");
  }

  int totalDispensed = 0;
  int remaining = target;

  while (remaining > 0) {
    int chunk = (remaining > MAX_PER_CUT) ? MAX_PER_CUT : remaining;

    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
    delay(5);

    int dispensed = 0;
    for (int i = 0; i < chunk; i++) {
      bool ok = feedOneResistor(M1_DIR, M1_STEP, s1_blocked, FEED_DIR,
                                FEED_MODE, FEED_DELAY_US, STEP_TIMEOUT);
      if (!ok) {
        Serial.print("! Timeout after "); Serial.print(totalDispensed + dispensed);
        Serial.println(" resistors (reel jam / empty?)");
        loadState = REMOVE;
        break;
      }
      dispensed++;
      Serial.print("Dispensed "); Serial.println(totalDispensed + dispensed);
    }

    stepMotor(M1_DIR, M1_STEP, -10);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);

    // -- cut the reel after this chunk (stepper already disabled) --
    if (dispensed > 0) cutReel();

    totalDispensed += dispensed;
    remaining      -= dispensed;

    if (dispensed < chunk) break;   // timeout occurred -> stop
  }

  return totalDispensed;
}


// -- Init Helpers ------------------------------------------------
void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser, OUTPUT); pinMode(rclk, OUTPUT); pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);
}

void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin, OUTPUT); pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, LOW); digitalWrite(stepPin, LOW);
}


// -- Serial ------------------------------------------------------
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("load")) {
    Serial.println("Loading...");
    LoadStatus s = loadResistor();
    Serial.print("Status: "); Serial.println(statusName(s));
    if (s == REMOVE)
      Serial.println("! Strip stuck / too short - remove and reload.");
    return;
  }

  if (line.equalsIgnoreCase("status")) {
    // refresh sensor reading for an at-rest EMPTY/LOADED report
    if (loadState != REMOVE)
      loadState = schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO) ? LOADED : EMPTY;
    Serial.print("Status: "); Serial.println(statusName(loadState));
    return;
  }

  int qty = line.toInt();
  if (qty <= 0) {
    Serial.println("! Enter a positive count (e.g. 2), or \"load\" / \"status\"");
    return;
  }

  Serial.print("Dispensing "); Serial.print(qty); Serial.println(" resistor(s)...");
  int done = dispenseResistors(qty);
  Serial.print("Finished. Total: "); Serial.println(done);
}


// -- Setup & Loop ------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(100);                 // let PCA9685 finish its own power-on reset

  initSR(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
  initMotorPins(M1_DIR, M1_STEP);
  pinMode(SENSOR_PIN, INPUT);

  disableAllMotors();

  // PCA9685 init
  i2cRecover();               // clear any stuck bus before starting I2C
  Wire.begin();               // pin20 (SDA), pin21 (SCL) on Mega
  Wire.setClock(100000);      // 100kHz - more forgiving than 400kHz
  pwm.begin();
  pwm.setPWMFreq(50);         // 50Hz for analog servos
  delay(10);

  // Park cutter at rest
  moveServo(CUT_CH_A, CUT_A_REST);
  moveServo(CUT_CH_B, CUT_B_REST);
  delay(CUT_TRAVEL_MS);

  schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO);
  loadState = s1_blocked ? LOADED : EMPTY;

  Serial.println("Resistor dispenser ready. Enter a number, \"load\", or \"status\".");
}

void loop() {
  handleSerial();
}