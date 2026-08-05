// ================================================================
//  Stepper only — DRV8825 via 74HC595 enable
//  Serial: enter <steps> (negative = reverse) → moves that many steps
//          "home" → move in HOME_DIR until limit switch reads LOW
// ================================================================

// ── Shift Register 1 ─────────────────────────────────────────────
#define SR1_SER    2
#define SR1_RCLK   3
#define SR1_SRCLK  4

// ── Motor 1 ──────────────────────────────────────────────────────
#define M1_STEP    32
#define M1_DIR     33
#define M1_EN_BIT  5

// ── Limit switch ─────────────────────────────────────────────────
#define LIMIT_PIN      0
#define HOME_TIMEOUT   200000   // max microsteps before giving up

// ── Step profile ─────────────────────────────────────────────────
#define STEP_MODE      32
#define STEP_DELAY_US  1600

// ── Direction config ─────────────────────────────────────────────
#define HOME_DIR      -1        // negative = reverse toward switch
#define SLOT_DIR       1        // direction to advance through slots

byte sr1 = 0xFF;

// ── Shift Register ───────────────────────────────────────────────
void shiftOut595(byte val) {
  digitalWrite(SR1_RCLK, LOW);
  shiftOut(SR1_SER, SR1_SRCLK, MSBFIRST, val);
  digitalWrite(SR1_RCLK, HIGH);
}

void enableMotor(bool enable) {
  if (!enable) sr1 |=  (1u << M1_EN_BIT);   // active-LOW: HIGH = disabled
  else         sr1 &= ~(1u << M1_EN_BIT);
  shiftOut595(sr1);
}

// ── Stepper ──────────────────────────────────────────────────────
void pulseStep(byte stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

// steps = magnitude, dir = +1 forward / -1 reverse
void stepMotor(byte dirPin, byte stepPin, long steps, int dir, int mode, int delayUs) {
  digitalWrite(dirPin, dir >= 0 ? HIGH : LOW);   // explicit direction
  long count = (long)abs(steps) * mode;

  enableMotor(true);
  delay(5);
  for (long i = 0; i < count; i++) pulseStep(stepPin, delayUs);
  // stay energized indefinitely (do NOT disable)
}

// ── Slot geometry ────────────────────────────────────────────────
#define SLOT1_OFFSET   14   // steps from home to slot 1
#define SLOT_PITCH     20   // steps between consecutive slots

// ── Home: move in HOME_DIR until switch reads LOW ────────────────
bool homeMotor(bool holdRelease = true) {
  // already home?
  if (digitalRead(LIMIT_PIN) == LOW) {
    enableMotor(true);   // hold position
    if (holdRelease) {
      delay(2000);       // energize 2 s
      enableMotor(false);
    }
    Serial.println("Already home.");
    return true;
  }

  digitalWrite(M1_DIR, HOME_DIR >= 0 ? HIGH : LOW);   // direction toward switch
  enableMotor(true);
  delay(5);

  long pulses = 0;
  bool ok = true;
  while (digitalRead(LIMIT_PIN) == HIGH) {
    pulseStep(M1_STEP, STEP_DELAY_US);
    if (++pulses > HOME_TIMEOUT) { ok = false; break; }
  }

  if (ok) {
    Serial.println("Homed. Switch triggered.");
    if (holdRelease) {
      delay(2000);        // stay energized 2 s
      enableMotor(false);
    }
    // else: leave energized for the follow-on move
  } else {
    enableMotor(false);
    Serial.println("! Home timeout (switch never triggered).");
  }
  return ok;
}

// ── Move to a slot: home only on slot 1, then incremental ────────
int currentSlot = 0;   // 0 = unknown / not homed

void moveToSlot(int slot) {
  if (slot == 1) {
    Serial.println("Homing for slot 1...");
    if (!homeMotor(false)) {        // home, stay energized
      Serial.println("! Aborting (home failed).");
      currentSlot = 0;
      return;
    }
    stepMotor(M1_DIR, M1_STEP, SLOT1_OFFSET, SLOT_DIR, STEP_MODE, STEP_DELAY_US);  // home → slot 1
    currentSlot = 1;
    Serial.print("At slot 1 ("); Serial.print(SLOT1_OFFSET); Serial.println(" steps).");
    return;
  }

  // slots 2..10: just move one pitch forward from wherever we are
  stepMotor(M1_DIR, M1_STEP, SLOT_PITCH, SLOT_DIR, STEP_MODE, STEP_DELAY_US);
  currentSlot = slot;
  Serial.print("At slot "); Serial.print(slot);
  Serial.print(" (+"); Serial.print(SLOT_PITCH); Serial.println(" steps).");
}

// ── Serial ───────────────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  cmd.toLowerCase();

  if (cmd == "home") {
    Serial.println("Homing...");
    homeMotor();
    return;
  }

  if (cmd.startsWith("slot")) {
    int slot = cmd.substring(4).toInt();
    if (slot < 1 || slot > 10) {
      Serial.println("! slot must be 1-10");
      return;
    }
    moveToSlot(slot);
    return;
  }

  long steps = line.toInt();
  if (steps == 0) {
    Serial.println("! Enter a non-zero step count (negative = reverse), or 'home'");
    return;
  }

  Serial.print("Stepping "); Serial.print(steps); Serial.println("...");
  stepMotor(M1_DIR, M1_STEP, steps, steps >= 0 ? 1 : -1, STEP_MODE, STEP_DELAY_US);  // sign sets dir
  Serial.println("Done.");
}

// ── Setup & Loop ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(SR1_SER, OUTPUT);
  pinMode(SR1_RCLK, OUTPUT);
  pinMode(SR1_SRCLK, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_STEP, OUTPUT);
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M1_STEP, LOW);

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  sr1 = 0xFF;
  shiftOut595(sr1);        // all disabled at boot

  Serial.println("Stepper ready. <steps> | home | slot <1-10>");
}

void loop() {
  handleSerial();
}