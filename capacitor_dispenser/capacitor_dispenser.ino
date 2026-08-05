// ================================================================
//  Capacitor Dispenser
//  DRV8825 Stepper (via 74HC595 enable) + analog photo-interrupter
//  + servo cutter
//  Enter a number in Serial Monitor → dispenses that many Capacitors
//  Sensor edge detection via software Schmitt trigger.
//  Cuts in batches of at most MAX_PER_CUT (default 2).
//  Commands: <number> | load | status
// ================================================================

#include <Servo.h>

// ── Shift Register 1 ─────────────────────────────────────────────
#define SR1_SER    5
#define SR1_RCLK   6
#define SR1_SRCLK  7

// ── Motor 1 ──────────────────────────────────────────────────────
#define M1_STEP    38
#define M1_DIR     39
#define M1_EN_BIT  0

// ── Photo-interrupter (Schmitt trigger) ──────────────────────────
#define SENSOR_PIN   A0
#define SENSOR_HI    900   // must rise above this → BLOCKED
#define SENSOR_LO    640    // must fall below this → CLEAR

// ── Dispense feed profile ────────────────────────────────────────
#define FEED_DIR         1
#define FEED_MODE        32
#define FEED_DELAY_US    400
#define STEP_TIMEOUT   20000

// ── Load feed profile ────────────────────────────────────────────
#define LOAD_DIR         1
#define LOAD_DELAY_US    100
#define LOAD_TIMEOUT   20000

// ── Batch cut limit ──────────────────────────────────────────────
#define MAX_PER_CUT      2    // never cut more than this many at once

// ── Cutter servo ─────────────────────────────────────────────────
#define SERVO_PIN      14
#define CUT_REST_ANGLE 80    // idle / retracted
#define CUT_ANGLE      35    // blade-down
#define CUT_TRAVEL_MS 400    // time to reach each end (tune)
#define CUT_HOLD_MS   250    // dwell at the cut before retracting

Servo cutter;

byte sr1 = 0xFF;

// ── Schmitt-trigger state ────────────────────────────────────────
bool s1_blocked = false;

// ── Load status flag ─────────────────────────────────────────────
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


// ── Shift Register Functions ─────────────────────────────────────
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


// ── Low-level Stepper ────────────────────────────────────────────
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

void disableAllMotors() {
  sr1 = 0xFF;                                  // all enable bits HIGH = disabled (active-LOW)
  shiftOut595(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
}

// ── Sensor: software Schmitt trigger ─────────────────────────────
bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) {
    if (v < lo) state = false;
  } else {
    if (v > hi) state = true;
  }
  return state;
}


// ── Cutter ───────────────────────────────────────────────────────
// One cut: rest → cut → rest.
void cutReel() {
  cutter.write(CUT_ANGLE);
  delay(CUT_TRAVEL_MS + CUT_HOLD_MS);
  cutter.write(CUT_REST_ANGLE);
  delay(CUT_TRAVEL_MS);
}


// ── Feed one Capacitor ────────────────────────────────────────────
bool feedOneCapacitor(byte dirPin, byte stepPin, bool& sensorState,
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


// ── Feed a batch, advance past sensor, then cut ──────────────────
// Feeds up to `count` capacitors, returns how many were actually fed.
int dispenseBatch(int count) {
  int fed = 0;
  for (int i = 0; i < count; i++) {
    bool ok = feedOneCapacitor(M1_DIR, M1_STEP, s1_blocked, FEED_DIR,
                               FEED_MODE, FEED_DELAY_US, STEP_TIMEOUT);
    if (!ok) {
      Serial.print("! Timeout in batch after "); Serial.print(fed);
      Serial.println(" Capacitors (reel jam / empty?)");
      loadState = REMOVE;
      break;
    }
    fed++;
    Serial.print("Dispensed "); Serial.println(fed);
  }

  if (fed > 0) {
    stepMotor(M1_DIR, M1_STEP, 600);   // advance past sensor before cutting
    Serial.print("Cutting batch of "); Serial.print(fed); Serial.println("...");
    cutReel();
    Serial.println("Cut done.");
  }

  return fed;
}


// ── High-level Dispense ──────────────────────────────────────────
// Splits qty into batches of at most MAX_PER_CUT, cutting each batch.
int dispenseCapacitors(int qty) {
  if (qty <= 0) return 0;

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  delay(5);

  int dispensed = 0;
  int remaining = qty;

  while (remaining > 0) {
    int thisBatch = (remaining > MAX_PER_CUT) ? MAX_PER_CUT : remaining;
    int fed = dispenseBatch(thisBatch);
    dispensed += fed;
    if (fed < thisBatch) break;   // jam/empty → stop
    remaining -= fed;
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);

  if (dispensed == 0) {
    Serial.println("Nothing dispensed - skipping cut.");
  }

  return dispensed;
}


// ── Load: feed strip until sensor detects, then stop immediately ─
bool loadStrip() {
  // already something at the sensor?
  if (schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO)) {
    loadState = LOADED;
    Serial.println("Already loaded (object at sensor).");
    return true;
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  delay(5);

  digitalWrite(M1_DIR, LOAD_DIR >= 0 ? HIGH : LOW);

  long pulses = 0;
  bool ok = true;
  while (!schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO)) {
    pulseStep(M1_STEP, LOAD_DELAY_US);
    if (++pulses > LOAD_TIMEOUT) { ok = false; break; }
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);

  if (ok) {
    loadState = LOADED;
    Serial.println("Loaded. Object detected at sensor.");
  } else {
    loadState = REMOVE;   // motor moved but no sensor change → jam/empty
    Serial.println("! Load timeout (no object detected).");
  }
  return ok;
}


// ── Status: report whether object is at the sensor ───────────────
void reportStatus() {
  int v = analogRead(SENSOR_PIN);
  bool detected = schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO);

  // sync flag with current reading (unless a load timeout latched REMOVE)
  if (detected)                  loadState = LOADED;
  else if (loadState != REMOVE)  loadState = EMPTY;

  Serial.print("Status: ");
  Serial.print(statusName(loadState));
  Serial.print(detected ? " (OBJECT DETECTED)" : " (clear)");
  Serial.print("  (raw="); Serial.print(v); Serial.println(")");
}


// ── Init Helpers ─────────────────────────────────────────────────
void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser, OUTPUT); pinMode(rclk, OUTPUT); pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);
}

void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin, OUTPUT); pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, LOW); digitalWrite(stepPin, LOW);
}


// ── Serial ───────────────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  cmd.toLowerCase();

  if (cmd == "load") {
    Serial.println("Loading strip...");
    loadStrip();
    return;
  }

  if (cmd == "status") {
    reportStatus();
    return;
  }

  int qty = line.toInt();
  if (qty <= 0) {
    Serial.println("! Enter a positive count, or 'load' / 'status'");
    return;
  }

  Serial.print("Dispensing "); Serial.print(qty); Serial.println(" Capacitor(s)...");
  int done = dispenseCapacitors(qty);
  Serial.print("Finished. Total: "); Serial.println(done);
}


// ── Setup & Loop ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  initSR(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
  initMotorPins(M1_DIR, M1_STEP);
  pinMode(SENSOR_PIN, INPUT);

  disableAllMotors();

  // cutter servo: attach and park at rest
  cutter.attach(SERVO_PIN, 500, 2500);
  cutter.write(CUT_REST_ANGLE);

  delay(100);

  // initial sensor read → seed the load flag
  bool detected = schmitt(SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO);
  loadState = detected ? LOADED : EMPTY;

  Serial.println("Capacitor dispenser ready. Commands: <number> | load | status");
}

void loop() {
  handleSerial();
}