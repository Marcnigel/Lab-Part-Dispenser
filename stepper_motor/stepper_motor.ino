// ================================================================
//  DRV8825 Stepper + 74HC595 Enable Control
//  Arduino Giga R1 WiFi
//
//  Wiring (this example):
//    74HC595 SR1:  SER → pin 2 | RCLK → pin 3 | SRCLK → pin 4
//    DRV8825 M1:   STEP → 22  | DIR → 23 | ENABLE → Q0 of SR1
//
//  To add more hardware, add defines + a state byte per SR,
//  then call init/enable/step with those new pins. No structs needed.
// ================================================================


// ── Pin Definitions ──────────────────────────────────────────────

// Shift Register 1
#define SR1_SER    2
#define SR1_RCLK   3
#define SR1_SRCLK  4

// Motor 1
#define M1_STEP    28
#define M1_DIR     29
#define M1_EN_BIT  3// Q0 of SR1  (Q1 → 1, Q2 → 2, etc.)

// ── Adding more hardware looks like this: ────────────────────────
// #define SR2_SER    5
// #define SR2_RCLK   6
// #define SR2_SRCLK  7
//
// #define M2_STEP    24
// #define M2_DIR     25
// #define M2_EN_BIT  0   // Q0 of SR2
//
// #define M3_STEP    26
// #define M3_DIR     27
// #define M3_EN_BIT  1   // Q1 of SR1  (reusing SR1 for a second motor)
// ─────────────────────────────────────────────────────────────────


// ── Shift Register Shadow States ─────────────────────────────────
// One byte per independent 74HC595.
// Tracks the current output so we can change one bit without
// disturbing the others (595 is write-only hardware).
// 0xFF = all outputs HIGH = all DRV8825 ENABLE lines HIGH = all disabled.

byte sr1 = 0xFF;
// byte sr2 = 0xFF;   // add one for each extra shift register


//Shift Register Functions
void shiftOut595(byte ser, byte rclk, byte srclk, byte val) {
  digitalWrite(rclk, LOW);
  shiftOut(ser, srclk, MSBFIRST, val);
  digitalWrite(rclk, HIGH);
}

// Flip one output bit (0-7) and latch.
// 'state' is passed by reference so the shadow stays in sync.
void setEnableBit(byte ser, byte rclk, byte srclk, byte& state, byte bit, bool value) {
  if (value) state |=  (1u << bit);
  else       state &= ~(1u << bit);
  shiftOut595(ser, rclk, srclk, state);
}

// enable=true  → coils energised (ENABLE pin driven LOW — active-LOW)
// enable=false → coils free      (ENABLE pin driven HIGH)
void enableMotor(byte ser, byte rclk, byte srclk, byte& state, byte bit, bool enable) {
  setEnableBit(ser, rclk, srclk, state, bit, !enable);
}


// ── Stepper Function ─────────────────────────────────────────────

// steps > 0 → clockwise  |  steps < 0 → counter-clockwise
// mode      → microstepping multiplier (1=full, 2=half, 8, 16, 32 …)
//             pulses sent = abs(steps) * mode
// delayUs   → LOW-time between pulses in microseconds (controls speed)

void stepMotor(byte dirPin, byte stepPin, int steps, int mode = 1, int delayUs = 800) {
  digitalWrite(dirPin, steps >= 0 ? HIGH : LOW);
  long count = (long)abs(steps) * mode;
  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
  }
}


// ── Init Helpers ─────────────────────────────────────────────────

void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser,   OUTPUT);
  pinMode(rclk,  OUTPUT);
  pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);  // latch safe state immediately
}

void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin,  OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin,  LOW);
  digitalWrite(stepPin, LOW);
}

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int steps = line.toInt();
  if (steps == 0) {
    Serial.println("! Enter a non-zero step count, e.g. 1000 or -1000");
    return;
  }

  Serial.print("M1 moving "); Serial.print(steps); Serial.println(" steps");

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);
  delay(5);

  stepMotor(M1_DIR, M1_STEP, steps);   // sign controls direction

  enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
  Serial.println("Done.");
}

// ── Setup & Loop ─────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);

  // Init shift registers first (motors stay disabled during pin setup)
  initSR(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1);
  // initSR(SR2_SER, SR2_RCLK, SR2_SRCLK, sr2);

  // Init motor direction/step pins
  initMotorPins(M1_DIR, M1_STEP);
  // initMotorPins(M2_DIR, M2_STEP);

  delay(100);

  // ── Example motion ───────────────────────────────────────────
  //enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, true);


  //Serial.println("200 steps CCW, 1/16 microstepping");
  //stepMotor(M1_DIR, M1_STEP, -200, 32, 200);
  //delay(300);

  //enableMotor(SR1_SER, SR1_RCLK, SR1_SRCLK, sr1, M1_EN_BIT, false);
 // Serial.println("Done. Motor disabled.");
}

void loop() {
  // your control logic here
  handleSerial();
}