// ================================================================
//  Combined Kit Dispenser  (Arduino Mega)
//
//  RA8875 touch UI  +  Resistor dispenser  +  Wire cutter (3 steppers)
//  +  Capacitor dispenser
//
//  Display mapping:
//    "1nF Capacitor" qty  -> number of capacitors to dispense
//    "Wire Set"      qty  -> number of SEQ_CYCLES for the wire cutter
//    "1k Resistor"   qty  -> number of resistors to dispense
//
//  Shift registers:
//     SR_A (2,3,4)  -> wire-cutter motors M1/M2/M3   (bits 0,1,2)
//     SR_B (5,6,7)  -> capacitor motor (bit 0), resistor motor (bit 3)
//
//  Cutters:
//     Resistor reel -> PCA9685 ch 0 / 1  (two servos)
//     Capacitor reel-> direct Servo on D14 (Timer5, no PCA conflict)
// ================================================================

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include <Adafruit_PWMServoDriver.h>


// ================================================================
//  DISPLAY (RA8875)
// ================================================================
#define RA8875_CS     8
#define RA8875_RESET  9
#define RA8875_INT    10
#define RA8875_DARKGREY 0x4208

#define X_MIN 58
#define X_MAX 960
#define Y_MIN 150
#define Y_MAX 916

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480

Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

const char* parts[] = {
  "Wire Set",
  "1k Resistor",
  "1nF Capacitor",
  "10k Resistor",
  "10nF Capacitor",
  "Red LED",
  "1N4148 Diode"
};

const uint8_t NUM_PARTS = 7;

#define IDX_WIRE   0
#define IDX_RES    1
#define IDX_CAP    2

int quantities[NUM_PARTS] = {0};

const bool partEnabled[NUM_PARTS] = {
    true,   // Wire Set
    true,   // 1k Resistor
    true,   // 1nF Capacitor
    false,  // 10k Resistor
    false,  // 10nF Capacitor
    false,  // Red LED
    false   // 1N4148 Diode
};

int numKits = 0;

const int startY = 68;
const int rowSpacing = 42;

const int minusX = 315;
const int quantityX = 375;
const int plusX = 450;

const int buttonW = 42;
const int buttonH = 34;

const int kitMinusX = 565;
const int kitPlusX  = 700;
const int kitButtonY = 140;


// ================================================================
//  WIRE CUTTER  (3 steppers, shift register SR_A on 2/3/4)
// ================================================================
#define SRA_SER    2
#define SRA_RCLK   3
#define SRA_SRCLK  4

#define M1_STEP    22
#define M1_DIR     23
#define M1_EN_BIT  0      // SR_A Q0

#define M2_STEP    24
#define M2_DIR     25
#define M2_EN_BIT  1      // SR_A Q1

#define M3_STEP    26
#define M3_DIR     27
#define M3_EN_BIT  2      // SR_A Q2

#define STEP_DELAY_US   800
#define SEQ_STEPS       5000
#define M3_SEQ_STEPS    2000

byte srA = 0xFF;


// ================================================================
//  SHIFT REGISTER B  (shared: capacitor + resistor motors)
// ================================================================
#define SRB_SER    5
#define SRB_RCLK   6
#define SRB_SRCLK  7

byte srB = 0xFF;


// ================================================================
//  RESISTOR DISPENSER  (SR_B bit 3 + PCA9685 ch 0/1)
// ================================================================
#define R_STEP     38
#define R_DIR      39
#define R_EN_BIT   3

#define SENSOR_PIN   A3
#define SENSOR_HI    950   // rise above -> BLOCKED
#define SENSOR_LO    350   // fall below  -> CLEAR

#define FEED_DIR         -1
#define FEED_MODE        32
#define FEED_DELAY_US    1600
#define STEP_TIMEOUT   20000

#define PCA_ADDR      0x40
#define SERVO_MIN     150
#define SERVO_MAX     600

#define CUT_CH_A      0
#define CUT_CH_B      1
#define CUT_A_REST    90
#define CUT_A_CUT     0
#define CUT_B_REST    0
#define CUT_B_CUT     90
#define CUT_TRAVEL_MS 400
#define CUT_HOLD_MS   300

#define SDA_PIN       20
#define SCL_PIN       21

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

bool s1_blocked = false;
bool firstRun = true;


// ================================================================
//  CAPACITOR DISPENSER  (SR_B bit 0 + direct Servo cutter)
// ================================================================
#define C_STEP     32
#define C_DIR      33
#define C_EN_BIT   0      // SR_B Q0

#define C_SENSOR_PIN   A0
#define C_SENSOR_HI    900   // rise above -> BLOCKED
#define C_SENSOR_LO    680   // fall below  -> CLEAR

#define C_FEED_DIR         1     // note: opposite of the resistor reel
#define C_FEED_MODE        32
#define C_FEED_DELAY_US    1600
#define C_STEP_TIMEOUT   20000
#define C_POST_STEPS     100     // advance past the sensor after last part

#define C_SERVO_PIN      14
#define C_CUT_REST_ANGLE 70
#define C_CUT_ANGLE       5
#define C_CUT_TRAVEL_MS 400
#define C_CUT_HOLD_MS   150

Servo capCutter;

bool c1_blocked = false;
bool capFirstRun = true;


// ================================================================
//  SHARED SHIFT-REGISTER HELPERS
// ================================================================
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

void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser, OUTPUT); pinMode(rclk, OUTPUT); pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);
}

void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin, OUTPUT); pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, LOW); digitalWrite(stepPin, LOW);
}


// ================================================================
//  STEPPER LOW-LEVEL
// ================================================================
void pulseStep(byte stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

void stepMotor(byte dirPin, byte stepPin, long steps, int mode = 1, int delayUs = STEP_DELAY_US) {
  digitalWrite(dirPin, steps >= 0 ? HIGH : LOW);
  long count = labs(steps) * mode;
  for (long i = 0; i < count; i++) pulseStep(stepPin, delayUs);
}

void stepMotorsSynced(byte dirPin1, byte stepPin1,
                      byte dirPin2, byte stepPin2,
                      long steps, int delayUs = STEP_DELAY_US) {
  bool dirHigh = (steps >= 0);
  digitalWrite(dirPin1, dirHigh ? HIGH : LOW);
  digitalWrite(dirPin2, dirHigh ? HIGH : LOW);
  long count = labs(steps);
  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(delayUs);
  }
}


// ================================================================
//  SENSOR (software Schmitt trigger)
// ================================================================
bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) { if (v < lo) state = false; }
  else       { if (v > hi) state = true;  }
  return state;
}


// ================================================================
//  PCA9685 SERVO / RESISTOR CUTTER
// ================================================================
int angleToPulse(int angle) { return map(angle, 0, 180, SERVO_MIN, SERVO_MAX); }
void moveServo(uint8_t ch, int angle) { pwm.setPWM(ch, 0, angleToPulse(angle)); }

void i2cRecover() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);
  if (digitalRead(SDA_PIN) == LOW) {
    pinMode(SCL_PIN, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
      digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
      if (digitalRead(SDA_PIN) == HIGH) break;
    }
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}

void cutReel() {
  moveServo(CUT_CH_A, CUT_A_CUT);
  moveServo(CUT_CH_B, CUT_B_CUT);
  delay(CUT_TRAVEL_MS + CUT_HOLD_MS);
  moveServo(CUT_CH_A, CUT_A_REST);
  moveServo(CUT_CH_B, CUT_B_REST);
  delay(CUT_TRAVEL_MS);
}


// ================================================================
//  CAPACITOR CUTTER (direct servo)
// ================================================================
void cutCapReel() {
  capCutter.write(C_CUT_ANGLE);
  delay(C_CUT_TRAVEL_MS + C_CUT_HOLD_MS);
  capCutter.write(C_CUT_REST_ANGLE);
  delay(C_CUT_TRAVEL_MS);
}


// ================================================================
//  GENERIC FEED-ONE-PART  (blocked edge -> clear edge)
// ================================================================
bool feedOnePart(byte dirPin, byte stepPin,
                 byte sensorPin, bool& sensorState, int hi, int lo,
                 int dir, int delayUs, long timeoutSteps) {
  digitalWrite(dirPin, dir >= 0 ? HIGH : LOW);
  long pulses = 0;

  schmitt(sensorPin, sensorState, hi, lo);

  while (!schmitt(sensorPin, sensorState, hi, lo)) {
    pulseStep(stepPin, delayUs);
    if (++pulses > timeoutSteps) return false;
  }
  while (schmitt(sensorPin, sensorState, hi, lo)) {
    pulseStep(stepPin, delayUs);
    if (++pulses > timeoutSteps) return false;
  }
  return true;
}


// ================================================================
//  RESISTOR DISPENSE
// ================================================================
int dispenseResistors(int qty) {
  if (qty <= 0) return 0;

  int target = qty;
  if (firstRun) {
    target += 1;
    firstRun = false;
    Serial.println("(first run: dispensing one extra resistor)");
  }

  enableMotor(SRB_SER, SRB_RCLK, SRB_SRCLK, srB, R_EN_BIT, true);
  delay(5);

  int dispensed = 0;
  for (int i = 0; i < target; i++) {
    bool ok = feedOnePart(R_DIR, R_STEP,
                          SENSOR_PIN, s1_blocked, SENSOR_HI, SENSOR_LO,
                          FEED_DIR, FEED_DELAY_US, STEP_TIMEOUT);
    if (!ok) {
      Serial.print("! Timeout after "); Serial.print(dispensed);
      Serial.println(" resistors (reel jam / empty?)");
      break;
    }
    dispensed++;
    Serial.print("Dispensed resistor "); Serial.println(dispensed);
  }

  stepMotor(R_DIR, R_STEP, -10);
  enableMotor(SRB_SER, SRB_RCLK, SRB_SRCLK, srB, R_EN_BIT, false);

  Serial.println("Cutting resistor reel...");
  cutReel();
  Serial.println("Cut done.");

  return dispensed;
}


// ================================================================
//  CAPACITOR DISPENSE
// ================================================================
int dispenseCapacitors(int qty) {
  if (qty <= 0) return 0;

  int target = qty;
  if (capFirstRun) {
    target += 1;
    capFirstRun = false;
    Serial.println("(first run: dispensing one extra capacitor)");
  }

  enableMotor(SRB_SER, SRB_RCLK, SRB_SRCLK, srB, C_EN_BIT, true);
  delay(5);

  int dispensed = 0;
  for (int i = 0; i < target; i++) {
    bool ok = feedOnePart(C_DIR, C_STEP,
                          C_SENSOR_PIN, c1_blocked, C_SENSOR_HI, C_SENSOR_LO,
                          C_FEED_DIR, C_FEED_DELAY_US, C_STEP_TIMEOUT);
    if (!ok) {
      Serial.print("! Timeout after "); Serial.print(dispensed);
      Serial.println(" capacitors (reel jam / empty?)");
      break;
    }
    dispensed++;
    Serial.print("Dispensed capacitor "); Serial.println(dispensed);
  }

  // advance clear of the sensor so the next batch starts on a fresh edge
  stepMotor(C_DIR, C_STEP, (long)C_FEED_DIR * C_POST_STEPS);
  enableMotor(SRB_SER, SRB_RCLK, SRB_SRCLK, srB, C_EN_BIT, false);

  Serial.println("Cutting capacitor reel...");
  cutCapReel();
  Serial.println("Cut done.");

  return dispensed;
}


// ================================================================
//  WIRE CUTTER SEQUENCE
// ================================================================
void runSequence(int cycles) {
  for (int c = 0; c < cycles; c++) {
    Serial.print("=== Wire cycle "); Serial.print(c + 1);
    Serial.print(" of "); Serial.print(cycles); Serial.println(" ===");

    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M3_EN_BIT, true);
    delay(5);
    stepMotor(M3_DIR, M3_STEP, -M3_SEQ_STEPS);
    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M3_EN_BIT, false);

    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M1_EN_BIT, true);
    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M2_EN_BIT, true);
    delay(5);
    stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP, -SEQ_STEPS);
    stepMotorsSynced(M1_DIR, M1_STEP, M2_DIR, M2_STEP,  SEQ_STEPS);
    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M1_EN_BIT, false);
    enableMotor(SRA_SER, SRA_RCLK, SRA_SRCLK, srA, M2_EN_BIT, false);

    Serial.println("Wire cycle done.");
  }
  Serial.println("=== All wire cycles complete ===");
}


// ================================================================
//  BATCH DISPENSE
// ================================================================
void runBatch() {
  int capQty  = quantities[IDX_CAP];
  int resQty  = quantities[IDX_RES];
  int wireCyc = quantities[IDX_WIRE];

  Serial.println();
  Serial.println("========== DISPENSE ==========");
  Serial.print("Kits: ");            Serial.println(numKits);
  Serial.print("Capacitors/kit: ");  Serial.println(capQty);
  Serial.print("Resistors/kit: ");   Serial.println(resQty);
  Serial.print("Wire cycles/kit: "); Serial.println(wireCyc);
  Serial.println("------------------------------");

  if (numKits <= 0) {
    Serial.println("! numKits is 0 — nothing to do.");
    return;
  }

  for (int k = 0; k < numKits; k++) {
    Serial.print(">>> KIT "); Serial.print(k + 1);
    Serial.print(" of "); Serial.print(numKits); Serial.println(" <<<");

    if (capQty > 0)  dispenseCapacitors(capQty);
    if (resQty > 0)  dispenseResistors(resQty);
    if (wireCyc > 0) runSequence(wireCyc);
  }

  Serial.println("========== BATCH COMPLETE ==========");
}


// ================================================================
//  UI DRAWING
// ================================================================
bool pointInRect(int px, int py, int x, int y, int w, int h) {
  return (px >= x && px <= (x + w) && py >= y && py <= (y + h));
}

void drawCenteredText(int x, int y, int w, int h, const char *text,
                      uint16_t textColor, uint16_t bgColor, uint8_t size) {
  int charWidth  = 8  * (size + 1);
  int charHeight = 16 * (size + 1);
  int textWidth  = strlen(text) * charWidth;
  int tx = x + (w - textWidth) / 2;
  int ty = y + (h - charHeight) / 2;
  tft.textMode();
  tft.textColor(textColor, bgColor);
  tft.textEnlarge(size);
  tft.textSetCursor(tx, ty);
  tft.textWrite(text);
  tft.graphicsMode();
}

void drawButton(int x, int y, int w, int h, uint16_t fillColor,
                uint16_t textColor, const char* label) {
  tft.fillRoundRect(x, y, w, h, 8, fillColor);
  tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);
  drawCenteredText(x, y, w, h, label, textColor, fillColor, 1);
}

void drawQuantityBox(int x, int y, int value) {
  char buffer[6];
  sprintf(buffer, "%d", value);
  tft.fillRect(x, y, 60, 40, RA8875_WHITE);
  tft.drawRect(x, y, 60, 40, RA8875_BLACK);
  drawCenteredText(x, y, 60, 40, buffer, RA8875_BLACK, RA8875_WHITE, 1);
}

void redrawQuantity(int index) {
  int rowY = startY + index * rowSpacing;
  drawQuantityBox(quantityX, rowY - 3, quantities[index]);
}

void redrawKitQuantity() {
  drawQuantityBox(625, 140, numKits);
}

void touchToScreen(uint16_t rawX, uint16_t rawY, int &x, int &y) {
  x = (int)((rawX - X_MIN) * (float)SCREEN_WIDTH  / (X_MAX - X_MIN));
  y = (int)((rawY - Y_MIN) * (float)SCREEN_HEIGHT / (Y_MAX - Y_MIN));
  if (x < 0) x = 0;  if (x > SCREEN_WIDTH)  x = SCREEN_WIDTH;
  if (y < 0) y = 0;  if (y > SCREEN_HEIGHT) y = SCREEN_HEIGHT;
}

void drawInterface() {
  tft.fillScreen(RA8875_BLACK);

  tft.fillRect(0, 0, 800, 50, RA8875_RED);
  drawCenteredText(0, 0, 800, 50,
    "ENSC120 Lab Component Selector", RA8875_WHITE, RA8875_RED, 2);

  for (int i = 0; i < NUM_PARTS; i++) {
    int rowY = startY + i * rowSpacing;
    tft.textMode();
    tft.textColor(RA8875_WHITE, RA8875_BLACK);
    tft.textEnlarge(1);
    tft.textSetCursor(20, rowY + 9);
    tft.textWrite(parts[i]);
    tft.graphicsMode();

    drawButton(315, rowY, 42, 34, RA8875_RED,   RA8875_WHITE, "-");
    drawQuantityBox(375, rowY - 3, quantities[i]);
    drawButton(450, rowY, 42, 34, RA8875_GREEN, RA8875_WHITE, "+");
  }

  tft.fillRect(550, 80, 220, 120, RA8875_DARKGREY);
  drawCenteredText(550, 85, 220, 40, "# of Kits",
                   RA8875_WHITE, RA8875_DARKGREY, 1);

  drawButton(565, 140, 45, 40, RA8875_RED,   RA8875_WHITE, "-");
  drawQuantityBox(625, 140, numKits);
  drawButton(700, 140, 45, 40, RA8875_GREEN, RA8875_WHITE, "+");

  drawButton(550, 300, 220, 90, RA8875_GREEN, RA8875_WHITE, "DISPENSE");
}


// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  delay(100);

  // Shift registers + motors
  initSR(SRA_SER, SRA_RCLK, SRA_SRCLK, srA);
  initSR(SRB_SER, SRB_RCLK, SRB_SRCLK, srB);
  initMotorPins(M1_DIR, M1_STEP);
  initMotorPins(M2_DIR, M2_STEP);
  initMotorPins(M3_DIR, M3_STEP);
  initMotorPins(R_DIR,  R_STEP);
  initMotorPins(C_DIR,  C_STEP);
  pinMode(SENSOR_PIN,   INPUT);
  pinMode(C_SENSOR_PIN, INPUT);

  // PCA9685 (resistor cutter)
  i2cRecover();
  Wire.begin();
  Wire.setClock(100000);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  moveServo(CUT_CH_A, CUT_A_REST);
  moveServo(CUT_CH_B, CUT_B_REST);

  // Capacitor cutter servo
  capCutter.attach(C_SERVO_PIN, 500, 2500);
  capCutter.write(C_CUT_REST_ANGLE);
  delay(CUT_TRAVEL_MS);

  schmitt(SENSOR_PIN,   s1_blocked, SENSOR_HI,   SENSOR_LO);
  schmitt(C_SENSOR_PIN, c1_blocked, C_SENSOR_HI, C_SENSOR_LO);

  // Display
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 not found");
    while (1);
  }
  tft.displayOn(true);
  tft.GPIOX(true);
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
  tft.PWM1out(255);
  tft.touchEnable(true);
  drawInterface();

  Serial.println("Kit dispenser ready.");
}


// ================================================================
//  LOOP  (touch handling)
// ================================================================
void loop() {
  uint16_t rawX, rawY;
  int x, y;
  static unsigned long lastTouchTime = 0;

  if (!tft.touched())              return;
  if (!tft.touchRead(&rawX, &rawY)) return;
  if (millis() - lastTouchTime < 300) return;
  lastTouchTime = millis();

  touchToScreen(rawX, rawY, x, y);

  for (int i = 0; i < NUM_PARTS; i++) {
    int rowY = startY + i * rowSpacing;

    if (pointInRect(x, y, minusX, rowY, buttonW, buttonH)) {
      if (partEnabled[i] && quantities[i] > 0) {
        quantities[i]--;
        redrawQuantity(i);
      }
      return;
    }
    if (pointInRect(x, y, plusX, rowY, buttonW, buttonH)) {
      if (partEnabled[i] && quantities[i] < 9) {
        quantities[i]++;
        redrawQuantity(i);
      }
      return;
    }
  }

  if (pointInRect(x, y, kitMinusX, kitButtonY, 45, 40)) {
    if (numKits > 0) { numKits--; redrawKitQuantity(); }
    return;
  }
  if (pointInRect(x, y, kitPlusX, kitButtonY, 45, 40)) {
    if (numKits < 9) { numKits++; redrawKitQuantity(); }
    return;
  }

  if (pointInRect(x, y, 550, 300, 220, 90)) {
    Serial.println("DISPENSED");
    runBatch();
    return;
  }
}