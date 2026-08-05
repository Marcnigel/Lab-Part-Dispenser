// ================================================================
//  5-Stepper Control: two synced + three independent
//  DRV8825 x5 + 74HC595 enable  +  limit switch (monitor + home)
//  Arduino Giga R1 WiFi
//
//  Hardware:
//    Motor 1 (M1): STEP 22 | DIR 23 | ENABLE → SR1 Q0
//    Motor 2 (M2): STEP 24 | DIR 25 | ENABLE → SR1 Q1
//    Motor 3 (M3): STEP 26 | DIR 27 | ENABLE → SR1 Q2 (bit 2)
//    Motor 4 (M4): STEP 28 | DIR 29 | ENABLE → SR1 Q3 (bit 3)
//    Motor 5 (M5): STEP 30 | DIR 31 | ENABLE → SR1 Q4 (bit 4)
//    Limit switch: PIN 11 → GND (INPUT_PULLUP, active-LOW)
//
//  M1 and M2 move together at FULL STEP. M3, M4, M5 move independently
//  at 1/32 microstep. Each independent motor has its own step-delay and
//  step-count so its microstep mode does not affect M1/M2.
//  The limit switch reports its state over serial and is used by the
//  "home" command to stop M1+M2 travel.
//
//  Serial usage (9600 baud, Newline ending):
//    1 <steps>   → move M1+M2 together      e.g.  1 200   or  1 -200
//    2 <steps|Ncm> → move M3   e.g.  2 50   or  2 2cm
//    3 <steps|Ncm> → move M4   e.g.  3 50   or  3 1.5cm
//    4 <steps|Ncm> → move M5   e.g.  4 50   or  4 5cm
//    home        → move M1+M2 in negative steps until limit switch is HIGH
//    start       → run the sequence
//
//  Distance calibration (M3/M4/M5): 10000 steps = 3.3 cm
//    → 3030.303 steps per cm (see STEPS_PER_CM).
// ================================================================


// ── Pin Definitions ──────────────────────────────────────────────

#define SR1_SER    2
#define SR1_RCLK   3
#define SR1_SRCLK  4

#define M1_STEP    22
#define M1_DIR     23
#define M1_EN_BIT  0      // SR1 Q0

#define M2_STEP    24
#define M2_DIR     25
#define M2_EN_BIT  1      // SR1 Q1

#define M3_STEP    26
#define M3_DIR     27
#define M3_EN_BIT  2      // SR1 Q2

#define M4_STEP    28
#define M4_DIR     29
#define M4_EN_BIT  3      // SR1 Q3

#define M5_STEP    30
#define M5_DIR     31
#define M5_EN_BIT  4      // SR1 Q4

#define SW_PIN     11     // limit switch to GND, active-LOW


// ── Tuning: M1 + M2 (FULL STEP MODE) ───────────────────────────────

// Delay increased: Full steps cover 32x more distance, so pulsing must be slower
// to maintain the same RPM. (Previous 200 * 16 ~ 3200us)
#define STEP_DELAY_US   3200   // µs between pulses — M1 + M2 speed (full step)

#define SEQ_CYCLES      2      // number of times "start" repeats the sequence

// Steps divided by 32: 5000 / 32 = 156 full steps
#define SEQ_STEPS       156    // M1+M2 step count used in the sequence


// ── Tuning: M3 / M4 / M5 (1/32 MICROSTEP MODE, independent) ─────────

// Independent motors run in 1/32 microstepping: 32x more pulses per revolution,
// so pulse faster (shorter delay) to keep a comparable RPM. Adjust only these
// to change each motor's speed.
#define M3_STEP_DELAY_US  100    // µs between pulses — M3 speed (1/32 microstep)
#define M4_STEP_DELAY_US  100    // µs between pulses — M4 speed (1/32 microstep)
#define M5_STEP_DELAY_US  100    // µs between pulses — M5 speed (1/32 microstep)

// Sequence step counts in 1/32 microsteps.
#define M3_SEQ_STEPS      5000   // M3 microsteps used in the sequence
#define M4_SEQ_STEPS      5000   // M4 microsteps used in the sequence
#define M5_SEQ_STEPS      5000   // M5 microsteps used in the sequence

#define SW_DEBOUNCE_MS  20     // limit switch debounce window


// ── Homing Config (M1 + M2) ────────────────────────────────────────
// "home" drives M1+M2 in the negative direction until SW_PIN reads HIGH.
#define HOME_MAX_STEPS   100000  // safety cap so homing can't run forever


// ── Distance Calibration (M3 / M4 / M5) ────────────────────────────
// Measured: -10000 steps = 3.3 cm  →  negative steps travel this distance.
// 10000 / 3.3 = 3030.303 steps per cm; sign is negative for travel.
#define STEPS_PER_CM   -3030.303f   // 1/32 microsteps per cm (negative direction)


// ── Shift Register Shadow State ───────────────────────────────────

byte sr1 = 0xFF;   // 0xFF = all outputs HIGH = all motors disabled


// ── Limit Switch State ────────────────────────────────────────────

int swLastState = HIGH;


// ── Opto Sensors (per-motor load detection) ───────────────────────
// Analog photo-interrupters, software Schmitt trigger (hi/lo thresholds).
//   A6 → M3 (command "2 2cm")   hi 340 / lo 310
//   A7 → M4 (command "3 2cm")   hi 470 / lo 440
//   A8 → M5 (command "4 3cm")   hi 370 / lo 340
// "blocked" (reading above hi) = object sensed at the beam.

#define S3_PIN   A6
#define S3_HI    340
#define S3_LO    310

#define S4_PIN   A7
#define S4_HI    470
#define S4_LO    440

#define S5_PIN   A8
#define S5_HI    370
#define S5_LO    340

// Load status per motor.
enum LoadStatus { EMPTY, INSTOCK };

LoadStatus load3State = EMPTY;   // M3
LoadStatus load4State = EMPTY;   // M4
LoadStatus load5State = EMPTY;   // M5

// Schmitt-trigger memory for each sensor.
bool s3_blocked = false;
bool s4_blocked = false;
bool s5_blocked = false;

const char* statusName(LoadStatus s) {
  switch (s) {
    case EMPTY:   return "EMPTY";
    case INSTOCK: return "INSTOCK";
  }
  return "?";
}

// Software Schmitt trigger: returns true when "blocked" (sensed).
bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) { if (v < lo) state = false; }
  else       { if (v > hi) state = true;  }
  return state;
}


// ── Shift Register Functions ──────────────────────────────────────

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
  setEnableBit(ser, rclk, srclk, state, bit, !enable);  // DRV8825 ENABLE is active-LOW
}


// ── Stepper Functions ─────────────────────────────────────────────

// Move ONE motor by 'steps' (used for the independent motors, M3/M4/M5)
void stepMotor(byte dirPin, byte stepPin, long steps, int delayUs = STEP_DELAY_US) {
  digitalWrite(dirPin, steps >= 0 ? HIGH : LOW);
  long count = labs(steps);
  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
  }
}

// Move TWO motors together, step-for-step, same direction, same timing.
void stepMotorsSynced(byte dirPin1, byte stepPin1,
                       byte dirPin2, byte stepPin2,
                       int steps, int delayUs = STEP_DELAY_US) {
  bool dirHigh = (steps >= 0);
  digitalWrite(dirPin1, dirHigh ? HIGH : LOW);
  digitalWrite(dirPin2, dirHigh ? HIGH : LOW);

  long count = abs(steps);
  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(delayUs);
  }
}


// ── Homing (M1 + M2) ──────────────────────────────────────────────
// Drives M1+M2 in the negative direction one step at a time until the
// limit switch reads HIGH, or until HOME_MAX_STEPS is reached (safety).

void homeM1M2() {
  Serial.println("Homing M1+M2 (negative until switch HIGH)...");

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, true);
  delay(5);

  // Negative direction
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  long moved = 0;
  bool reached = false;

  while (moved < HOME_MAX_STEPS) {
    if (digitalRead(SW_PIN) == HIGH) { reached = true; break; }

    digitalWrite(M1_STEP, HIGH);
    digitalWrite(M2_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(M1_STEP, LOW);
    digitalWrite(M2_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);

    moved++;
  }

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, false);

  swLastState = digitalRead(SW_PIN);   // resync switch monitor

  if (reached) {
    Serial.print("Home reached after "); Serial.print(moved);
    Serial.println(" steps (switch HIGH).");
  } else {
    Serial.print("! Home aborted: HOME_MAX_STEPS ("); Serial.print(HOME_MAX_STEPS);
    Serial.println(") reached without switch HIGH.");
  }
}


// ── Wire Cut (M1 + M2) ────────────────────────────────────────────
// Moves M1+M2 up 650 steps, then homes (negative until switch HIGH).

void wire_cut() {
  Serial.println("Wire cut: M1+M2 up 650 steps...");

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, true);
  delay(5);

  stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP, 650);

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, false);

  homeM1M2();
  Serial.println("Wire cut done.");
}


// ── Load Functions (M3 / M4 / M5) ──────────────────────────────────
// cm-based moves (split to stay under the 16-bit int step limit).
// STEPS_PER_CM is negative, so "N cm forward" = N * STEPS_PER_CM steps.

// Move one independent motor a given cm amount (own enable bit + delay).
void moveMotorCm(byte dirPin, byte stepPin, byte enBit, int delayUs, float cm) {
  long steps = lroundf(cm * STEPS_PER_CM);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, enBit, true);
  delay(5);
  stepMotor(dirPin, stepPin, steps, delayUs);
  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, enBit, false);
}

// load 2 → M3: +10cm, +4cm, cut, -4cm
void load2() {
  Serial.println("load 2 (M3): +10cm, +4cm, cut, -4cm");
  moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, 10);
  moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, 4);
  wire_cut();
  moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, -4);
  Serial.println("load 2 done.");
}

// load 3 → M4: +9cm, cut, -4cm
void load3() {
  Serial.println("load 3 (M4): +9cm, cut, -4cm");
  moveMotorCm(M4_DIR, M4_STEP, M4_EN_BIT, M4_STEP_DELAY_US, 9);
  wire_cut();
  moveMotorCm(M4_DIR, M4_STEP, M4_EN_BIT, M4_STEP_DELAY_US, -4);
  Serial.println("load 3 done.");
}

// load 4 → M5: +10cm, +7cm, cut, -4cm
void load4() {
  Serial.println("load 4 (M5): +10cm, +7cm, cut, -4cm");
  moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, 10);
  moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, 7);
  wire_cut();
  moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, -4);
  Serial.println("load 4 done.");
}

// load → run all three in order
void loadAll() {
  Serial.println("=== load: running load2, load3, load4 ===");
  load2();
  load3();
  load4();
  Serial.println("=== load complete ===");
}


// ── Status Reporting (per-motor opto sensor) ──────────────────────
// Reads the sensor directly: sensed -> INSTOCK, not sensed -> EMPTY.
// No longer depends on the load command.
void reportStatus(byte pin, bool& sState, int hi, int lo,
                  LoadStatus& flag, const char* label) {
  bool sensed = schmitt(pin, sState, hi, lo);
  flag = sensed ? INSTOCK : EMPTY;
  Serial.print(label); Serial.print(" status: ");
  Serial.println(statusName(flag));
}


// ── Init Helpers ──────────────────────────────────────────────────

void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser,   OUTPUT);
  pinMode(rclk,  OUTPUT);
  pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);
}

void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin,  OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin,  LOW);
  digitalWrite(stepPin, LOW);
}


// ── Sequence ──────────────────────────────────────────────────────
// One cycle: M3 -M3_SEQ_STEPS, M4 -M4_SEQ_STEPS, M5 -M5_SEQ_STEPS,
//            then M1+M2 -SEQ_STEPS, then M1+M2 +SEQ_STEPS.

void runSequence(int cycles) {
  for (int c = 0; c < cycles; c++) {
    Serial.print("=== Cycle "); Serial.print(c + 1);
    Serial.print(" of "); Serial.print(cycles); Serial.println(" ===");

    // M3 → -M3_SEQ_STEPS  (1/32 microstep, own delay)
    Serial.print("M3 moving "); Serial.print(-M3_SEQ_STEPS); Serial.println(" steps");
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M3_EN_BIT, true);
    delay(5);
    stepMotor(M3_DIR, M3_STEP, -M3_SEQ_STEPS, M3_STEP_DELAY_US);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M3_EN_BIT, false);

    // M4 → -M4_SEQ_STEPS  (1/32 microstep, own delay)
    Serial.print("M4 moving "); Serial.print(-M4_SEQ_STEPS); Serial.println(" steps");
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M4_EN_BIT, true);
    delay(5);
    stepMotor(M4_DIR, M4_STEP, -M4_SEQ_STEPS, M4_STEP_DELAY_US);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M4_EN_BIT, false);

    // M5 → -M5_SEQ_STEPS  (1/32 microstep, own delay)
    Serial.print("M5 moving "); Serial.print(-M5_SEQ_STEPS); Serial.println(" steps");
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M5_EN_BIT, true);
    delay(5);
    stepMotor(M5_DIR, M5_STEP, -M5_SEQ_STEPS, M5_STEP_DELAY_US);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M5_EN_BIT, false);

    // M1+M2 → -SEQ_STEPS  (full step)
    Serial.print("M1+M2 moving "); Serial.print(-SEQ_STEPS); Serial.println(" steps");
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, true);
    delay(5);
    stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP, -SEQ_STEPS);

    // M1+M2 → +SEQ_STEPS  (full step)
    Serial.print("M1+M2 moving "); Serial.print(SEQ_STEPS); Serial.println(" steps");
    stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP, SEQ_STEPS);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, false);

    Serial.println("Cycle done.");
  }
  Serial.println("=== All cycles complete ===");
}


// ── Limit Switch Handling ─────────────────────────────────────────
// Detects a debounced state change and reports it over serial.
// Monitor only — takes no motor action.

void handleSwitch() {
  int state = digitalRead(SW_PIN);
  if (state != swLastState) {
    delay(SW_DEBOUNCE_MS);
    state = digitalRead(SW_PIN);
    if (state != swLastState) {
      Serial.print("Limit switch: ");
      Serial.println(state == LOW ? "PRESSED (LOW)" : "RELEASED (HIGH)");
      swLastState = state;
    }
  }
}


// ── Serial Command Handling ───────────────────────────────────────
// Accepted input:
//    start          → run the SEQ_CYCLES sequence
//    home           → home M1+M2 (negative until switch HIGH)
//    <group> <steps>  e.g.  "1 200"  or  "2 -50"

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // --- start command ---
  if (line.equalsIgnoreCase("start")) {
    Serial.print("Starting "); Serial.print(SEQ_CYCLES);
    Serial.println("-cycle sequence...");
    runSequence(SEQ_CYCLES);
    return;
  }

  // --- home command ---
  if (line.equalsIgnoreCase("home")) {
    homeM1M2();
    return;
  }

  // --- cut command ---
  if (line.equalsIgnoreCase("cut")) {
    wire_cut();
    return;
  }

  // --- load commands ---
  if (line.equalsIgnoreCase("load")) {
    loadAll();
    return;
  }
  if (line.equalsIgnoreCase("load 2")) { load2(); return; }
  if (line.equalsIgnoreCase("load 3")) { load3(); return; }
  if (line.equalsIgnoreCase("load 4")) { load4(); return; }

  // --- status commands (per-motor opto sensor) ---
  if (line.equalsIgnoreCase("status 2")) {
    reportStatus(S3_PIN, s3_blocked, S3_HI, S3_LO, load3State, "M3");
    return;
  }
  if (line.equalsIgnoreCase("status 3")) {
    reportStatus(S4_PIN, s4_blocked, S4_HI, S4_LO, load4State, "M4");
    return;
  }
  if (line.equalsIgnoreCase("status 4")) {
    reportStatus(S5_PIN, s5_blocked, S5_HI, S5_LO, load5State, "M5");
    return;
  }

  // --- manual move commands ---
  int spaceIdx = line.indexOf(' ');
  if (spaceIdx == -1) {
    Serial.println("! Usage: 1 <steps> | 2 <steps|Ncm> | 3 <steps|Ncm> | 4 <steps|Ncm> | home | start");
    return;
  }

  int    group = line.substring(0, spaceIdx).toInt();
  String arg   = line.substring(spaceIdx + 1);
  arg.trim();

  // Detect a "cm" suffix on the argument (groups 2/3/4 only).
  // e.g.  "2 2cm"  →  2 cm  →  round(2 * STEPS_PER_CM) steps
  int steps;
  bool inCm = false;
  {
    String a = arg;
    a.toLowerCase();
    if (a.endsWith("cm")) {
      inCm = true;
      float cm = arg.substring(0, arg.length() - 2).toFloat();
      steps = (int)lroundf(cm * STEPS_PER_CM);
    } else {
      steps = arg.toInt();
    }
  }

  if (inCm && group == 1) {
    Serial.println("! cm units are only valid for groups 2, 3, 4");
    return;
  }

  if (steps == 0) {
    Serial.println("! steps cannot be 0");
    return;
  }

  if (group == 1) {
    Serial.print("M1+M2 moving "); Serial.print(steps); Serial.println(" steps");

    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, true);
    delay(5);

    stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP, steps);

    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
    enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M2_EN_BIT, false);

    Serial.println("Done.");

  } else if (group == 2) {
    if (inCm) {
      float cm = steps / STEPS_PER_CM;   // recover requested cm
      Serial.print("M3 load: +4cm, +"); Serial.print(cm, 3); Serial.println("cm, cut, -4cm");
      moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, 4);
      moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, cm);
      wire_cut();
      moveMotorCm(M3_DIR, M3_STEP, M3_EN_BIT, M3_STEP_DELAY_US, -4);
      Serial.println("Done.");
    } else {
      Serial.print("M3 moving "); Serial.print(steps); Serial.println(" steps");
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M3_EN_BIT, true);
      delay(5);
      stepMotor(M3_DIR, M3_STEP, steps, M3_STEP_DELAY_US);
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M3_EN_BIT, false);
      Serial.println("Done.");
    }

  } else if (group == 3) {
    if (inCm) {
      float cm = steps / STEPS_PER_CM;
      Serial.print("M4 load: +4cm, +"); Serial.print(cm, 3); Serial.println("cm, cut, -4cm");
      moveMotorCm(M4_DIR, M4_STEP, M4_EN_BIT, M4_STEP_DELAY_US, 4);
      moveMotorCm(M4_DIR, M4_STEP, M4_EN_BIT, M4_STEP_DELAY_US, cm);
      wire_cut();
      moveMotorCm(M4_DIR, M4_STEP, M4_EN_BIT, M4_STEP_DELAY_US, -4);
      Serial.println("Done.");
    } else {
      Serial.print("M4 moving "); Serial.print(steps); Serial.println(" steps");
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M4_EN_BIT, true);
      delay(5);
      stepMotor(M4_DIR, M4_STEP, steps, M4_STEP_DELAY_US);
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M4_EN_BIT, false);
      Serial.println("Done.");
    }

  } else if (group == 4) {
    if (inCm) {
      float cm = steps / STEPS_PER_CM;
      Serial.print("M5 load: +4cm, +"); Serial.print(cm, 3); Serial.println("cm, cut, -4cm");
      moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, 4);
      moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, cm);
      wire_cut();
      moveMotorCm(M5_DIR, M5_STEP, M5_EN_BIT, M5_STEP_DELAY_US, -4);
      Serial.println("Done.");
    } else {
      Serial.print("M5 moving "); Serial.print(steps); Serial.println(" steps");
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M5_EN_BIT, true);
      delay(5);
      stepMotor(M5_DIR, M5_STEP, steps, M5_STEP_DELAY_US);
      enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M5_EN_BIT, false);
      Serial.println("Done.");
    }

  } else {
    Serial.println("! Unknown group. Use 1 (M1+M2), 2 (M3), 3 (M4), or 4 (M5)");
  }
}


// ── Setup & Loop ──────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  while (!Serial);

  initSR(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
  initMotorPins(M1_DIR, M1_STEP);
  initMotorPins(M2_DIR, M2_STEP);
  initMotorPins(M3_DIR, M3_STEP);
  initMotorPins(M4_DIR, M4_STEP);
  initMotorPins(M5_DIR, M5_STEP);

  pinMode(SW_PIN, INPUT_PULLUP);
  swLastState = digitalRead(SW_PIN);

  // Opto sensors (analog inputs)
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);
  schmitt(S3_PIN, s3_blocked, S3_HI, S3_LO);
  schmitt(S4_PIN, s4_blocked, S4_HI, S4_LO);
  schmitt(S5_PIN, s5_blocked, S5_HI, S5_LO);

  Serial.println("Ready.");
  Serial.println("1 <steps> = move M1+M2 together (full step)");
  Serial.println("2 <steps|Ncm> = move M3 (1/32 microstep)  e.g. 2 2cm");
  Serial.println("3 <steps|Ncm> = move M4 (1/32 microstep)  e.g. 3 1.5cm");
  Serial.println("4 <steps|Ncm> = move M5 (1/32 microstep)  e.g. 4 5cm");
  Serial.println("home      = move M1+M2 negative until switch HIGH");
  Serial.println("cut       = M1+M2 up 650 steps then home");
  Serial.println("load      = run load2, load3, load4 in order");
  Serial.println("load 2    = M3: +10cm,+4cm,cut,-4cm");
  Serial.println("load 3    = M4: +9cm,cut,-4cm");
  Serial.println("load 4    = M5: +10cm,+7cm,cut,-4cm");
  Serial.println("status 2/3/4 = report M3/M4/M5 sensor (EMPTY/INSTOCK)");
  Serial.println("start     = run sequence");
  Serial.println("(cal: 10000 steps = 3.3 cm -> 3030.303 steps/cm)");
}

void loop() {
  handleSerial();
  handleSwitch();
}