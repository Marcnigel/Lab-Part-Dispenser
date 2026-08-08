// ================================================================
//  Unified Kit Dispenser — Touchscreen UI + Motor Control
//  Arduino Mega 2560 R3 + RA8875 800x480
//
//  Integrates:
//    * 3 Resistor dispensers  (DRV8825 + PCA9685 dual-servo cutter)
//    * 3 Capacitor dispensers (DRV8825 + hobby-servo cutter)
//    * 3 Wire cutters         (M3/M4/M5 feed + M1/M2 guillotine cut)
//
//  Shift registers:
//    SR_RC   (5/6/7)  -> 6 resistor+capacitor motor enable bits (0..5)
//    SR_WIRE (2/3/4)  -> 5 wire-cutter enable bits (M1..M5, bits 0..4)
//                        + packing-system motor enable (bit 5)
//
//  Packing system:
//    On DISPENSE, the carousel homes, then advances one slot per kit.
//    For each slot it dispenses that kit's selected parts. The packing
//    motor stays energized for the whole run and is released only when
//    every kit is dispensed. # of Kits (1..10) = slots to fill.
//
//  UI mapping:
//    Dispense page:  each +/- box sets a per-slot quantity.
//        Resistor 1..3 / Capacitor 1..3 -> count  (x numKits)
//        Wire 1..3                       -> length in cm (x numKits)
//      DISPENSE runs every slot with quantity>0.
//    Load page:  per-slot Load button drives that slot's load routine
//        and recolours by status. "Load Resistors and Capacitors"
//        loads every R/C slot that isn't already LOADED.
// ================================================================

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

// =================================================================
//  DISPLAY
// =================================================================
#define RA8875_CS 8
#define RA8875_RESET 9
#define RA8875_INT 10
#define RA8875_DARKGREY 0x4208

#define X_MIN 58
#define X_MAX 960
#define Y_MIN 150
#define Y_MAX 916

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 480

Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

#define COLOR_BG RA8875_BLACK
#define COLOR_TAB_ACTIVE 0xFBE0
#define COLOR_TAB_INACTIVE 0xCB20
#define COLOR_PLUS 0xFBE0
#define COLOR_MINUS 0xCB20
#define COLOR_WHITE RA8875_WHITE
#define COLOR_TEXT RA8875_BLACK
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_LOADED COLOR_MINUS

// =================================================================
//  SHIFT REGISTERS
// =================================================================
// R/C dispensers share one SR (enable bits 0..5)
#define SRRC_SER 5
#define SRRC_RCLK 6
#define SRRC_SRCLK 7

// Wire cutter shares its own SR (M1..M5 enable bits)
#define SRW_SER 2
#define SRW_RCLK 3
#define SRW_SRCLK 4

byte srRC = 0xFF;  // all disabled (active-LOW)
byte srW = 0xFF;

// =================================================================
//  CAPACITOR DISPENSERS (3)  — hobby-servo cutter each
// =================================================================
#define CAP_FEED_DIR 1
#define CAP_FEED_MODE 32
#define CAP_FEED_DELAY_US 400
#define CAP_STEP_TIMEOUT 20000
#define CAP_LOAD_DIR 1
#define CAP_LOAD_DELAY_US 100
#define CAP_LOAD_TIMEOUT 20000
#define CAP_MAX_PER_CUT 2
#define CAP_ADVANCE_STEPS 600  // advance past sensor before cutting

#define CUT_REST_ANGLE 80
#define CUT_ANGLE 35
#define CUT_TRAVEL_MS 400
#define CUT_HOLD_MS 250

struct CapDispenser {
  byte stepPin, dirPin, enBit;
  byte sensorPin;
  int hi, lo;
  byte servoPin;
  bool blocked;  // schmitt state
  Servo cutter;
};

CapDispenser cap[3] = {
  // STEP DIR ENBIT  SENSOR HI   LO   SERVO
  { 38, 39, 0, A0, 900, 640, 14, false, Servo() },
  { 40, 41, 1, A1, 900, 620, 15, false, Servo() },
  { 42, 43, 2, A2, 900, 610, 16, false, Servo() },
};

// =================================================================
//  RESISTOR DISPENSERS (3)  — PCA9685 dual-servo cutter each
// =================================================================
#define RES_FEED_DIR 1
#define RES_FEED_MODE 32
#define RES_FEED_DELAY_US 100
#define RES_STEP_TIMEOUT 30000
#define RES_STALL_STEPS 9000
#define RES_MAX_PER_CUT 2
#define RES_BACKOFF_STEPS 10  // small reverse before cut

#define PCA_ADDR 0x40
#define SERVO_MIN 150
#define SERVO_MAX 600
#define RCUT_A_REST 90
#define RCUT_A_CUT 0
#define RCUT_B_REST 0
#define RCUT_B_CUT 100
#define RCUT_TRAVEL_MS 400
#define RCUT_HOLD_MS 300
#define SDA_PIN 20
#define SCL_PIN 21

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

struct ResDispenser {
  byte stepPin, dirPin, enBit;
  byte sensorPin;
  int hi, lo;
  byte cutChA, cutChB;
  bool blocked;
  bool needExtra;
};

ResDispenser res[3] = {
  // STEP DIR ENBIT  SENSOR HI   LO   CHA CHB
  { 44, 45, 3, A3, 900, 640, 0, 1, false, false },
  { 46, 47, 4, A4, 900, 440, 2, 3, false, false },
  { 48, 49, 5, A5, 900, 580, 4, 5, false, false },
};

// =================================================================
//  WIRE CUTTERS (3)  — feed motors M3/M4/M5, cut with M1/M2
// =================================================================
#define WM1_STEP 22
#define WM1_DIR 23
#define WM1_EN_BIT 0
#define WM2_STEP 24
#define WM2_DIR 25
#define WM2_EN_BIT 1

// Feed motors for wire 1/2/3 -> M3/M4/M5
#define WM3_STEP 26
#define WM3_DIR 27
#define WM3_EN_BIT 2
#define WM4_STEP 28
#define WM4_DIR 29
#define WM4_EN_BIT 3
#define WM5_STEP 30
#define WM5_DIR 31
#define WM5_EN_BIT 4

#define SW_PIN 11
#define SW_DEBOUNCE_MS 20
#define WIRE_STEP_DELAY_US 3200  // M1+M2 full-step
#define WIRE_FEED_DELAY_US 100   // M3/M4/M5 microstep
#define HOME_MAX_STEPS 100000
#define STEPS_PER_CM -3030.303f

// wire feed opto sensors (stock detection)
#define WS3_PIN A6
#define WS3_HI 340
#define WS3_LO 310
#define WS4_PIN A7
#define WS4_HI 470
#define WS4_LO 440
#define WS5_PIN A8
#define WS5_HI 370
#define WS5_LO 340

struct WireCutter {
  byte feedStep, feedDir, feedEnBit;
  byte sensorPin;
  int hi, lo;
  bool blocked;
};

WireCutter wire[3] = {
  // FEED_STEP DIR ENBIT  SENSOR HI  LO
  { WM3_STEP, WM3_DIR, WM3_EN_BIT, WS3_PIN, WS3_HI, WS3_LO, false },  // Wire1 -> M3
  { WM4_STEP, WM4_DIR, WM4_EN_BIT, WS4_PIN, WS4_HI, WS4_LO, false },  // Wire2 -> M4
  { WM5_STEP, WM5_DIR, WM5_EN_BIT, WS5_PIN, WS5_HI, WS5_LO, false },  // Wire3 -> M5
};

int swLastState = HIGH;

// =================================================================
//  PACKING SYSTEM (carousel)  — DRV8825 on SRW enable bit 5
// =================================================================
//  Shares the wire-cutter shift register (SRW, pins 2/3/4).
//  SRW enable bits: 0..4 = wire cutters (M1..M5), 5 = packing motor.
//  Its own STEP/DIR pins and a limit switch for homing.
#define PACK_STEP 32
#define PACK_DIR 33
#define PACK_EN_BIT 5     // bit 5 on SRW
#define PACK_LIMIT_PIN 0  // limit switch (normally-closed, reads LOW at home)

#define PACK_STEP_MODE 32
#define PACK_STEP_DELAY_US 1600
#define PACK_HOME_TIMEOUT 200000  // max microsteps before giving up
#define PACK_HOME_DIR -1          // negative = reverse toward switch
#define PACK_SLOT_DIR 1           // direction to advance through slots
#define PACK_SLOT1_OFFSET 14      // steps from home to slot 1
#define PACK_SLOT_PITCH 20        // steps between consecutive slots

int packCurrentSlot = 0;  // 0 = unknown / not homed

// =================================================================
//  DOOR SENSORS (Left / Right)
// =================================================================
#define DOOR_LEFT_PIN 12
#define DOOR_RIGHT_PIN 13
// Reed/magnetic switches to GND, using the internal pull-up:
//   LOW  = magnet present -> door CLOSED
//   HIGH = magnet absent  -> door OPEN
// If your switches are wired the opposite way, just flip the
// comparison inside readDoorOpen() below.

enum DoorState { DOOR_CLOSED,
                 DOOR_OPEN };
DoorState leftDoorState = DOOR_CLOSED;
DoorState rightDoorState = DOOR_CLOSED;

bool readDoorOpen(byte pin) {
  return digitalRead(pin) == HIGH;
}

// =================================================================
//  UI STATE
// =================================================================
enum Page { PAGE_DISPENSE,
            PAGE_RELOAD };
Page currentPage = PAGE_DISPENSE;
bool popupVisible = false;
bool dispensing = false;  // true while the full-screen "Dispensing in progress..." popup is up

// Sensor polling keeps the LOAD page synchronized with the physical sensors.
// A Schmitt trigger is already used for noise immunity; this interval limits
// display traffic while still reacting quickly when material is removed.
#define SENSOR_STATUS_POLL_MS 100

const char* parts[] = {
  "Wire 1(cm)", "Wire 2(cm)", "Wire 3(cm)",
  "Capacitor 1", "Capacitor 2", "Capacitor 3",
  "Resistor 1", "Resistor 2", "Resistor 3"
};
const char* wireLoadLabel[3] = { "Wire 1", "Wire 2", "Wire 3" };

const uint8_t NUM_PARTS = 9;

// Index layout: 0..2 wire, 3..5 capacitor, 6..8 resistor
const uint8_t partGridCol[NUM_PARTS] = { 0, 1, 2, 0, 0, 0, 2, 2, 2 };
const uint8_t partGridRow[NUM_PARTS] = { 0, 0, 0, 1, 2, 3, 1, 2, 3 };

int quantities[NUM_PARTS] = { 0 };

const bool partEnabled[NUM_PARTS] = {
  true, true, true, true, true, true, true, true, true
};

enum PartType { TYPE_WIRE,
                TYPE_CAPACITOR,
                TYPE_RESISTOR };
const PartType partType[NUM_PARTS] = {
  TYPE_WIRE, TYPE_WIRE, TYPE_WIRE,
  TYPE_CAPACITOR, TYPE_CAPACITOR, TYPE_CAPACITOR,
  TYPE_RESISTOR, TYPE_RESISTOR, TYPE_RESISTOR
};

// Map a part index to its dispenser array slot (0..2). Wires 0..2, caps 0..2, res 0..2.
inline int slotOf(int i) {
  if (partType[i] == TYPE_WIRE) return i;           // 0..2
  if (partType[i] == TYPE_CAPACITOR) return i - 3;  // 0..2
  return i - 6;                                     // resistor 0..2
}

enum DispenserFlag { FLAG_EMPTY,
                     FLAG_LOADED,
                     FLAG_REMOVE };
DispenserFlag dispenserFlag[NUM_PARTS] = {
  FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY,
  FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY,
  FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY
};

enum WireStock { WIRE_EMPTY,
                 WIRE_INSTOCK };
WireStock wireStock[NUM_PARTS] = {
  WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY,
  WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY,
  WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY
};

int numKits = 0;

// =================================================================
//  LAYOUT CONSTANTS
// =================================================================
const int TAB_H = 50;
const int colX[3] = { 20, 190, 360 };
const int rowY[4] = { 70, 170, 270, 370 };
const int BTN_W = 42;
const int BTN_H = 54;
const int QTY_W = 60;
const int QTY_H = 40;
const int GRID_QTY_H = 60;
const int GAP = 6;
const int LABEL_H = 34;
const int LABEL_TO_BUTTON_OFFSET = 36;

const int kitPanelX = 560;
const int kitPanelW = 230;
const int kitPanelH = 125;
const int KIT_BTN_W = 45;
const int KIT_BTN_H = BTN_H;
const int kitMinusX = kitPanelX + (kitPanelW - (KIT_BTN_W + GAP + QTY_W + GAP + KIT_BTN_W)) / 2;
const int kitQtyX = kitMinusX + KIT_BTN_W + GAP;
const int kitPlusX = kitQtyX + QTY_W + GAP;

const int dispenseBtnX = 560;
const int dispenseBtnW = 230;
const int dispenseBtnH = 110;  // reduced to make room for the door status boxes below

const int doorBoxW = (dispenseBtnW - GAP) / 2;
const int doorBoxH = 70;
const int leftDoorBoxX = dispenseBtnX;
const int rightDoorBoxX = dispenseBtnX + doorBoxW + GAP;

const int RELOAD_BTN_W = BTN_W + GAP + QTY_W + GAP + BTN_W;
const int RELOAD_BTN_H = 50;

const int reloadAllBtnX = dispenseBtnX;
const int reloadAllBtnW = dispenseBtnW;
const int reloadAllBtnH = 200;  // Load page button keeps its original full size

// Right-column vertical centering: on the Dispense page, the kit panel +
// DISPENSE button + door boxes are stacked and centered as one group in
// the space below the tab bar; on the Load page, the wire-status legend +
// Load-all button are centered the same way, as its own (differently
// sized) group.
const int RIGHT_COL_GAP = 15;  // vertical gap between stacked right-column blocks

const int dispenseGroupH = kitPanelH + RIGHT_COL_GAP + dispenseBtnH + RIGHT_COL_GAP + doorBoxH;
const int dispenseGroupY = TAB_H + ((SCREEN_HEIGHT - TAB_H) - dispenseGroupH) / 2;
const int kitPanelY = dispenseGroupY;
const int kitButtonY = kitPanelY + 60;  // +/- row stays in the same spot relative to panel top
const int dispenseBtnY = kitPanelY + kitPanelH + RIGHT_COL_GAP;
const int doorBoxY = dispenseBtnY + dispenseBtnH + RIGHT_COL_GAP;

const int reloadGroupH = kitPanelH + RIGHT_COL_GAP + reloadAllBtnH;
const int reloadGroupY = TAB_H + ((SCREEN_HEIGHT - TAB_H) - reloadGroupH) / 2;
const int legendY = reloadGroupY;
const int reloadAllBtnY = legendY + kitPanelH + RIGHT_COL_GAP;

const int WIRE_DOT_RADIUS = 6;
const int WIRE_DOT_MARGIN = 10;

const int legendX = kitPanelX;
const int legendW = kitPanelW;
const int legendH = kitPanelH;
const int legendDotX = legendX + 24;
const int legendRow1Y = legendY + 50;
const int legendRow2Y = legendY + 95;
const int legendTextX = legendDotX + 18;

const int POPUP_W = 420;
const int POPUP_H = 200;
const int popupX = (SCREEN_WIDTH - POPUP_W) / 2;
const int popupY = (SCREEN_HEIGHT - POPUP_H) / 2;
const int POPUP_CLOSE_SIZE = 32;
const int popupCloseX = popupX + POPUP_W - POPUP_CLOSE_SIZE - 10;
const int popupCloseY = popupY + 10;

// =================================================================
//  POSITION HELPERS
// =================================================================
int partMinusX(int i) {
  return colX[partGridCol[i]];
}
int partQtyX(int i) {
  return partMinusX(i) + BTN_W + GAP;
}
int partPlusX(int i) {
  return partQtyX(i) + QTY_W + GAP;
}
int partLabelY(int i) {
  return rowY[partGridRow[i]];
}
int partButtonY(int i) {
  return partLabelY(i) + LABEL_TO_BUTTON_OFFSET;
}
int partReloadBtnX(int i) {
  return colX[partGridCol[i]];
}

bool pointInRect(int px, int py, int x, int y, int w, int h) {
  return (px >= x && px <= (x + w) && py >= y && py <= (y + h));
}

// forward declarations for draw helpers
void drawQuantityBox(int x, int y, int w, int h, int value);
void drawReloadButton(int i);
void drawInterface();
void drawDoorBox(int x, int y, int w, int h, const char* label, DoorState state);

void redrawQuantity(int index) {
  drawQuantityBox(partQtyX(index), partButtonY(index) - 3, QTY_W, GRID_QTY_H, quantities[index]);
}
void redrawKitQuantity() {
  drawQuantityBox(kitQtyX, kitButtonY, QTY_W, GRID_QTY_H, numKits);
}

// =================================================================
//  LOW-LEVEL SHIFT REGISTER / STEPPER (shared)
// =================================================================
void shiftOut595(byte ser, byte rclk, byte srclk, byte val) {
  digitalWrite(rclk, LOW);
  shiftOut(ser, srclk, MSBFIRST, val);
  digitalWrite(rclk, HIGH);
}
void setEnableBit(byte ser, byte rclk, byte srclk, byte& state, byte bit, bool value) {
  if (value) state |= (1u << bit);
  else state &= ~(1u << bit);
  shiftOut595(ser, rclk, srclk, state);
}
// active-LOW enable
void enRC(byte bit, bool enable) {
  setEnableBit(SRRC_SER, SRRC_RCLK, SRRC_SRCLK, srRC, bit, !enable);
}
void enW(byte bit, bool enable) {
  setEnableBit(SRW_SER, SRW_RCLK, SRW_SRCLK, srW, bit, !enable);
}

void pulseStep(byte stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}
void stepMotor(byte dirPin, byte stepPin, long steps, int mode, int delayUs) {
  digitalWrite(dirPin, steps >= 0 ? HIGH : LOW);
  long count = labs(steps) * mode;
  for (long i = 0; i < count; i++) pulseStep(stepPin, delayUs);
}
void stepMotorsSynced(byte d1, byte s1, byte d2, byte s2, long steps, int delayUs) {
  bool dh = (steps >= 0);
  digitalWrite(d1, dh ? HIGH : LOW);
  digitalWrite(d2, dh ? HIGH : LOW);
  long count = labs(steps);
  for (long i = 0; i < count; i++) {
    digitalWrite(s1, HIGH);
    digitalWrite(s2, HIGH);
    delayMicroseconds(2);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    delayMicroseconds(delayUs);
  }
}

bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) {
    if (v < lo) state = false;
  } else {
    if (v > hi) state = true;
  }
  return state;
}

// =================================================================
//  PACKING SYSTEM LOGIC
// =================================================================
// active-LOW enable on SRW bit 5
void packEnable(bool enable) {
  enW(PACK_EN_BIT, enable);
}

// Home: move in PACK_HOME_DIR until limit switch reads LOW.
// Leaves the motor ENERGIZED (caller is responsible for disabling).
bool packHome() {
  packEnable(true);
  delay(5);

  // already home?
  if (digitalRead(PACK_LIMIT_PIN) == LOW) {
    packCurrentSlot = 0;
    return true;
  }

  digitalWrite(PACK_DIR, PACK_HOME_DIR >= 0 ? HIGH : LOW);
  long pulses = 0;
  bool ok = true;
  while (digitalRead(PACK_LIMIT_PIN) == HIGH) {
    pulseStep(PACK_STEP, PACK_STEP_DELAY_US);
    if (++pulses > PACK_HOME_TIMEOUT) {
      ok = false;
      break;
    }
  }
  packCurrentSlot = 0;
  return ok;
}

// Advance a magnitude of steps in PACK_SLOT_DIR (stays energized).
void packStep(long steps) {
  digitalWrite(PACK_DIR, PACK_SLOT_DIR >= 0 ? HIGH : LOW);
  long count = labs(steps) * PACK_STEP_MODE;
  for (long i = 0; i < count; i++) pulseStep(PACK_STEP, PACK_STEP_DELAY_US);
}

// Move to slot 1..10. Slot 1 = SLOT1_OFFSET from home; each further slot
// is one PITCH from the previous. Assumes packHome() already ran and the
// motor is energized. Incremental from packCurrentSlot.
void packMoveToSlot(int slot) {
  if (slot == 1) {
    packStep(PACK_SLOT1_OFFSET);
  } else {
    packStep(PACK_SLOT_PITCH);
  }
  packCurrentSlot = slot;
}

// =================================================================
//  CAPACITOR LOGIC
// =================================================================
void capCut(int s) {
  cap[s].cutter.write(CUT_ANGLE);
  delay(CUT_TRAVEL_MS + CUT_HOLD_MS);
  cap[s].cutter.write(CUT_REST_ANGLE);
  delay(CUT_TRAVEL_MS);
}

bool capFeedOne(int s) {
  CapDispenser& d = cap[s];
  digitalWrite(d.dirPin, CAP_FEED_DIR >= 0 ? HIGH : LOW);
  long pulses = 0;
  schmitt(d.sensorPin, d.blocked, d.hi, d.lo);
  while (!schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    pulseStep(d.stepPin, CAP_FEED_DELAY_US);
    if (++pulses > CAP_STEP_TIMEOUT) return false;
  }
  while (schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    pulseStep(d.stepPin, CAP_FEED_DELAY_US);
    if (++pulses > CAP_STEP_TIMEOUT) return false;
  }
  return true;
}

int capDispenseBatch(int s, int count) {
  CapDispenser& d = cap[s];
  int fed = 0;
  for (int i = 0; i < count; i++) {
    if (!capFeedOne(s)) {
      dispenserFlag[3 + s] = FLAG_REMOVE;
      break;
    }
    fed++;
  }
  if (fed > 0) {
    stepMotor(d.dirPin, d.stepPin, CAP_ADVANCE_STEPS, 1, CAP_FEED_DELAY_US);
    capCut(s);
  }
  return fed;
}

int capDispense(int s, int qty) {
  if (qty <= 0) return 0;
  enRC(cap[s].enBit, true);
  delay(5);
  int dispensed = 0, remaining = qty;
  while (remaining > 0) {
    int thisBatch = (remaining > CAP_MAX_PER_CUT) ? CAP_MAX_PER_CUT : remaining;
    int fed = capDispenseBatch(s, thisBatch);
    dispensed += fed;
    if (fed < thisBatch) break;
    remaining -= fed;
  }
  enRC(cap[s].enBit, false);
  return dispensed;
}

bool capLoad(int s) {
  CapDispenser& d = cap[s];
  if (schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    dispenserFlag[3 + s] = FLAG_LOADED;
    return true;
  }
  enRC(d.enBit, true);
  delay(5);
  digitalWrite(d.dirPin, CAP_LOAD_DIR >= 0 ? HIGH : LOW);
  long pulses = 0;
  bool ok = true;
  while (!schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    pulseStep(d.stepPin, CAP_LOAD_DELAY_US);
    if (++pulses > CAP_LOAD_TIMEOUT) {
      ok = false;
      break;
    }
  }
  enRC(d.enBit, false);
  dispenserFlag[3 + s] = ok ? FLAG_LOADED : FLAG_REMOVE;
  return ok;
}

// =================================================================
//  RESISTOR LOGIC
// =================================================================
int angleToPulse(int a) {
  return map(a, 0, 180, SERVO_MIN, SERVO_MAX);
}
void moveServoCh(uint8_t ch, int angle) {
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

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

void resCut(int s) {
  moveServoCh(res[s].cutChA, RCUT_A_CUT);
  moveServoCh(res[s].cutChB, RCUT_B_CUT);
  delay(RCUT_TRAVEL_MS + RCUT_HOLD_MS);
  moveServoCh(res[s].cutChA, RCUT_A_REST);
  moveServoCh(res[s].cutChB, RCUT_B_REST);
  delay(RCUT_TRAVEL_MS);
}

bool resFeedOne(int s) {
  ResDispenser& d = res[s];
  digitalWrite(d.dirPin, RES_FEED_DIR >= 0 ? HIGH : LOW);
  long pulses = 0;
  schmitt(d.sensorPin, d.blocked, d.hi, d.lo);
  while (!schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    pulseStep(d.stepPin, RES_FEED_DELAY_US);
    if (++pulses > RES_STEP_TIMEOUT) return false;
  }
  while (schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    pulseStep(d.stepPin, RES_FEED_DELAY_US);
    if (++pulses > RES_STEP_TIMEOUT) return false;
  }
  return true;
}

int resDispense(int s, int qty) {
  if (qty <= 0) return 0;
  ResDispenser& d = res[s];
  int target = qty;
  if (d.needExtra) {
    target += 2;
    d.needExtra = false;
  }

  int totalDispensed = 0, remaining = target;
  while (remaining > 0) {
    int chunk = (remaining > RES_MAX_PER_CUT) ? RES_MAX_PER_CUT : remaining;
    enRC(d.enBit, true);
    delay(5);
    int dispensed = 0;
    for (int i = 0; i < chunk; i++) {
      if (!resFeedOne(s)) {
        dispenserFlag[6 + s] = FLAG_REMOVE;
        break;
      }
      dispensed++;
    }
    stepMotor(d.dirPin, d.stepPin, -RES_BACKOFF_STEPS, 1, RES_FEED_DELAY_US);
    enRC(d.enBit, false);
    if (dispensed > 0) resCut(s);
    totalDispensed += dispensed;
    remaining -= dispensed;
    if (dispensed < chunk) break;
  }
  return totalDispensed;
}

bool resLoad(int s) {
  ResDispenser& d = res[s];
  if (schmitt(d.sensorPin, d.blocked, d.hi, d.lo)) {
    dispenserFlag[6 + s] = FLAG_LOADED;
    d.needExtra = true;
    return true;
  }
  enRC(d.enBit, true);
  delay(5);
  digitalWrite(d.dirPin, RES_FEED_DIR >= 0 ? HIGH : LOW);
  long sinceChange = 0;
  bool prev = d.blocked, ok = false;
  while (true) {
    pulseStep(d.stepPin, RES_FEED_DELAY_US);
    bool now = schmitt(d.sensorPin, d.blocked, d.hi, d.lo);
    if (now != prev) {
      prev = now;
      sinceChange = 0;
      if (now) {
        ok = true;
        break;
      }
    } else if (++sinceChange > RES_STALL_STEPS) {
      ok = false;
      break;
    }
  }
  enRC(d.enBit, false);
  if (ok) {
    dispenserFlag[6 + s] = FLAG_LOADED;
    d.needExtra = true;
  } else {
    dispenserFlag[6 + s] = FLAG_REMOVE;
  }
  return ok;
}

// =================================================================
//  WIRE CUTTER LOGIC
// =================================================================
void homeM1M2() {
  enW(WM1_EN_BIT, true);
  enW(WM2_EN_BIT, true);
  delay(5);
  digitalWrite(WM1_DIR, LOW);
  digitalWrite(WM2_DIR, LOW);
  long moved = 0;
  while (moved < HOME_MAX_STEPS) {
    if (digitalRead(SW_PIN) == HIGH) break;
    digitalWrite(WM1_STEP, HIGH);
    digitalWrite(WM2_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(WM1_STEP, LOW);
    digitalWrite(WM2_STEP, LOW);
    delayMicroseconds(WIRE_STEP_DELAY_US);
    moved++;
  }
  enW(WM1_EN_BIT, false);
  enW(WM2_EN_BIT, false);
  swLastState = digitalRead(SW_PIN);
}

void wireGuillotine() {
  enW(WM1_EN_BIT, true);
  enW(WM2_EN_BIT, true);
  delay(5);
  stepMotorsSynced(WM1_DIR, WM1_STEP, WM2_DIR, WM2_STEP, 675, WIRE_STEP_DELAY_US);
  enW(WM1_EN_BIT, false);
  enW(WM2_EN_BIT, false);
  delay(200);
  homeM1M2();
}

// Feed one wire slot forward N cm, cut, retract 4cm. Length in cm.
void wireFeedMotorCm(int s, float cm) {
  WireCutter& w = wire[s];
  long steps = lroundf(cm * STEPS_PER_CM);
  enW(w.feedEnBit, true);
  delay(5);
  stepMotor(w.feedDir, w.feedStep, steps, 1, WIRE_FEED_DELAY_US);
  enW(w.feedEnBit, false);
}

// Dispense a wire of `lengthCm`: feed lead-in, feed length, cut, retract.
void wireDispense(int s, float lengthCm) {
  if (lengthCm <= 0) return;
  wireFeedMotorCm(s, 4);         // lead-in
  wireFeedMotorCm(s, lengthCm);  // requested length
  wireGuillotine();              // M1/M2 cut + home
  wireFeedMotorCm(s, -4);        // retract
}

void wireUpdateStock(int s) {
  WireCutter& w = wire[s];
  bool sensed = schmitt(w.sensorPin, w.blocked, w.hi, w.lo);
  wireStock[s] = sensed ? WIRE_INSTOCK : WIRE_EMPTY;
}

// Keep all load indicators synchronized with their sensors while the machine
// is idle. FLAG_REMOVE is intentionally preserved until the operator retries;
// normal EMPTY/LOADED states follow the current physical sensor state.
void updateLoadStatusFromSensors() {
  static unsigned long lastPollMs = 0;
  unsigned long now = millis();
  if (now - lastPollMs < SENSOR_STATUS_POLL_MS) return;
  lastPollMs = now;

  // Door sensors - keep the Dispense page boxes live
  DoorState newLeft = readDoorOpen(DOOR_LEFT_PIN) ? DOOR_OPEN : DOOR_CLOSED;
  DoorState newRight = readDoorOpen(DOOR_RIGHT_PIN) ? DOOR_OPEN : DOOR_CLOSED;
  bool leftDoorChanged = (newLeft != leftDoorState);
  bool rightDoorChanged = (newRight != rightDoorState);
  leftDoorState = newLeft;
  rightDoorState = newRight;
  if (currentPage == PAGE_DISPENSE && !popupVisible) {
    if (leftDoorChanged)
      drawDoorBox(leftDoorBoxX, doorBoxY, doorBoxW, doorBoxH, "L Door", leftDoorState);
    if (rightDoorChanged)
      drawDoorBox(rightDoorBoxX, doorBoxY, doorBoxW, doorBoxH, "R Door", rightDoorState);
  }

  for (int s = 0; s < 3; s++) {
    // Capacitor sensor -> part indexes 3..5
    int capIndex = 3 + s;
    bool capPresent = schmitt(cap[s].sensorPin, cap[s].blocked,
                              cap[s].hi, cap[s].lo);
    if (dispenserFlag[capIndex] != FLAG_REMOVE) {
      DispenserFlag nextFlag = capPresent ? FLAG_LOADED : FLAG_EMPTY;
      if (dispenserFlag[capIndex] != nextFlag) {
        dispenserFlag[capIndex] = nextFlag;
        if (currentPage == PAGE_RELOAD && !popupVisible) {
          drawReloadButton(capIndex);
        }
      }
    }

    // Resistor sensor -> part indexes 6..8
    int resIndex = 6 + s;
    bool resPresent = schmitt(res[s].sensorPin, res[s].blocked,
                              res[s].hi, res[s].lo);
    if (dispenserFlag[resIndex] != FLAG_REMOVE) {
      DispenserFlag nextFlag = resPresent ? FLAG_LOADED : FLAG_EMPTY;
      if (dispenserFlag[resIndex] != nextFlag) {
        dispenserFlag[resIndex] = nextFlag;
        if (currentPage == PAGE_RELOAD && !popupVisible) {
          drawReloadButton(resIndex);
        }
      }
    }

    // Wire stock dots also follow their physical sensors continuously.
    WireStock previousStock = wireStock[s];
    wireUpdateStock(s);
    if (wireStock[s] != previousStock && currentPage == PAGE_RELOAD && !popupVisible) {
      drawReloadButton(s);
    }
  }
}

// "Load" for a wire slot just runs a feed/cut cycle and refreshes stock.
// Per-slot wire LOAD feed length (cm), indexed by wire slot 0..2
const float WIRE_LOAD_CM[3] = { 10, 7, 10 };  // set each dispenser's value

void wireLoad(int s) {
  wireDispense(s, WIRE_LOAD_CM[s]);  // token feed to verify strip present
  wireUpdateStock(s);
}

// =================================================================
//  UI-FACING ACTIONS
// =================================================================
void reloadPart(int index) {
  int s = slotOf(index);
  switch (partType[index]) {
    case TYPE_WIRE: wireLoad(s); break;
    case TYPE_CAPACITOR: capLoad(s); break;
    case TYPE_RESISTOR: resLoad(s); break;
  }
}

void removeExtra(int index) {
  int s = slotOf(index);
  // Re-run the load routine to clear the strip; sets LOADED/REMOVE as appropriate
  if (partType[index] == TYPE_CAPACITOR) capLoad(s);
  else if (partType[index] == TYPE_RESISTOR) resLoad(s);
}

// Load every resistor + capacitor slot that isn't already LOADED.
void loadResistorsAndCapacitors() {
  for (int i = 0; i < NUM_PARTS; i++) {
    if (partType[i] == TYPE_WIRE) continue;
    if (dispenserFlag[i] == FLAG_LOADED) continue;
    reloadPart(i);
  }
}

// Resistor/capacitor slots must be LOADED to dispense; wire slots aren't
// gated by dispenserFlag. Returns false if any selected (quantity > 0)
// resistor/capacitor slot is EMPTY or in the REMOVE (unable to load) state.
bool selectedDispensersReady() {
  for (int i = 0; i < NUM_PARTS; i++) {
    if (quantities[i] <= 0) continue;
    if (partType[i] == TYPE_WIRE) continue;
    if (dispenserFlag[i] != FLAG_LOADED) return false;
  }
  return true;
}

// Dispense one kit's worth of every selected part into the current slot.
// quantities[i] is the per-kit amount for that part.
void dispenseOneKit() {
  for (int i = 0; i < NUM_PARTS; i++) {
    int qty = quantities[i];
    if (qty <= 0) continue;
    int s = slotOf(i);
    if (partType[i] == TYPE_CAPACITOR) capDispense(s, qty);
    else if (partType[i] == TYPE_RESISTOR) resDispense(s, qty);
    else if (partType[i] == TYPE_WIRE) wireDispense(s, (float)qty);  // cm per kit
  }
}

// Full run: home the packing carousel, then for each kit move to the next
// slot and dispense that kit's parts. The packing motor stays ENERGIZED
// for the entire run and is only released once every kit is dispensed.
//
// numKits = number of packing slots to fill (1..10).
void runDispense() {
  // 1) Home the packing system first (motor energized on SRW bit 5).
  if (!packHome()) {
    packEnable(false);  // release on failure
    return;
  }

  // 2) One kit per slot: move to slot N, dispense kit N.
  for (int kit = 1; kit <= numKits; kit++) {
    packMoveToSlot(kit);  // slot 1 = home+offset, then +pitch each time
    dispenseOneKit();     // packing motor held energized throughout
  }

  // 3) All kits done — now it is safe to release the packing motor.
  packEnable(false);
}

// =================================================================
//  TOUCH / TEXT / DRAW HELPERS
// =================================================================
void touchToScreen(uint16_t rawX, uint16_t rawY, int& x, int& y) {
  x = (int)((rawX - X_MIN) * (float)SCREEN_WIDTH / (X_MAX - X_MIN));
  y = (int)((rawY - Y_MIN) * (float)SCREEN_HEIGHT / (Y_MAX - Y_MIN));
  if (x < 0) x = 0;
  if (x > SCREEN_WIDTH) x = SCREEN_WIDTH;
  if (y < 0) y = 0;
  if (y > SCREEN_HEIGHT) y = SCREEN_HEIGHT;
}

void drawCenteredText(int x, int y, int w, int h, const char* text,
                      uint16_t textColor, uint16_t bgColor, uint8_t size) {
  int charWidth = 8 * (size + 1);
  int charHeight = 16 * (size + 1);
  int textWidth = strlen(text) * charWidth;
  int tx = x + (w - textWidth) / 2;
  int ty = y + (h - charHeight) / 2;
  tft.textMode();
  tft.textColor(textColor, bgColor);
  tft.textEnlarge(size);
  tft.textSetCursor(tx, ty);
  tft.textWrite(text);
  tft.graphicsMode();
}

void drawLeftText(int x, int y, const char* text,
                  uint16_t textColor, uint16_t bgColor, uint8_t size) {
  int charHeight = 16 * (size + 1);
  tft.textMode();
  tft.textColor(textColor, bgColor);
  tft.textEnlarge(size);
  tft.textSetCursor(x, y - charHeight / 2);
  tft.textWrite(text);
  tft.graphicsMode();
}

void drawButton(int x, int y, int w, int h, uint16_t fillColor,
                uint16_t textColor, const char* label, uint8_t textSize = 1) {
  tft.fillRoundRect(x, y, w, h, 8, fillColor);
  tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);
  drawCenteredText(x, y, w, h, label, textColor, fillColor, textSize);
}

void drawButtonTwoLine(int x, int y, int w, int h, uint16_t fillColor,
                       uint16_t textColor, const char* line1, const char* line2, uint8_t size) {
  tft.fillRoundRect(x, y, w, h, 8, fillColor);
  tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);
  int lineHeight = 16 * (size + 1);
  int lineGap = 6;
  int blockH = lineHeight * 2 + lineGap;
  int startY = y + (h - blockH) / 2;
  drawCenteredText(x, startY, w, lineHeight, line1, textColor, fillColor, size);
  drawCenteredText(x, startY + lineHeight + lineGap, w, lineHeight, line2, textColor, fillColor, size);
}

void drawDoorBox(int x, int y, int w, int h, const char* label, DoorState state) {
  uint16_t fillColor = (state == DOOR_OPEN) ? COLOR_RED : COLOR_GREEN;
  const char* stateText = (state == DOOR_OPEN) ? "OPEN" : "CLOSED";
  drawButtonTwoLine(x, y, w, h, fillColor, RA8875_WHITE, label, stateText, 1);
}

void drawDoorBoxes() {
  drawDoorBox(leftDoorBoxX, doorBoxY, doorBoxW, doorBoxH, "L Door", leftDoorState);
  drawDoorBox(rightDoorBoxX, doorBoxY, doorBoxW, doorBoxH, "R Door", rightDoorState);
}

void drawQuantityBox(int x, int y, int w, int h, int value) {
  char buffer[6];
  sprintf(buffer, "%d", value);
  tft.fillRect(x, y, w, h, RA8875_WHITE);
  tft.drawRect(x, y, w, h, RA8875_BLACK);
  drawCenteredText(x, y, w, h, buffer, RA8875_BLACK, RA8875_WHITE, 1);
}

void drawTabBar() {
  uint16_t dispenseColor = (currentPage == PAGE_DISPENSE) ? COLOR_TAB_ACTIVE : COLOR_TAB_INACTIVE;
  uint16_t reloadColor = (currentPage == PAGE_RELOAD) ? COLOR_TAB_ACTIVE : COLOR_TAB_INACTIVE;
  tft.fillRect(0, 0, SCREEN_WIDTH / 2, TAB_H, dispenseColor);
  tft.fillRect(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H, reloadColor);
  drawCenteredText(0, 0, SCREEN_WIDTH / 2, TAB_H, "DISPENSE", COLOR_WHITE, dispenseColor, 2);
  drawCenteredText(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H, "LOAD", COLOR_WHITE, reloadColor, 2);
}

void drawDispensePage() {
  for (int i = 0; i < NUM_PARTS; i++) {
    int labelY = partLabelY(i);
    int minusX = partMinusX(i);
    int qtyX = partQtyX(i);
    int plusX = partPlusX(i);
    int buttonY = partButtonY(i);
    int blockW = (plusX + BTN_W) - minusX;
    drawCenteredText(minusX, labelY, blockW, LABEL_H, parts[i], RA8875_WHITE, RA8875_BLACK, 1);
    drawButton(minusX, buttonY, BTN_W, BTN_H, COLOR_MINUS, RA8875_WHITE, "-");
    drawQuantityBox(qtyX, buttonY - 3, QTY_W, GRID_QTY_H, quantities[i]);
    drawButton(plusX, buttonY, BTN_W, BTN_H, COLOR_PLUS, RA8875_WHITE, "+");
  }
  tft.fillRect(kitPanelX, kitPanelY, kitPanelW, kitPanelH, RA8875_DARKGREY);
  drawCenteredText(kitPanelX, kitPanelY + 5, kitPanelW, 40, "# of Kits", RA8875_WHITE, RA8875_DARKGREY, 1);
  drawButton(kitMinusX, kitButtonY, KIT_BTN_W, KIT_BTN_H, COLOR_MINUS, RA8875_WHITE, "-");
  drawQuantityBox(kitQtyX, kitButtonY, QTY_W, GRID_QTY_H, numKits);
  drawButton(kitPlusX, kitButtonY, KIT_BTN_W, KIT_BTN_H, COLOR_PLUS, RA8875_WHITE, "+");
  drawButton(dispenseBtnX, dispenseBtnY, dispenseBtnW, dispenseBtnH, COLOR_PLUS, RA8875_WHITE, "DISPENSE");
  drawDoorBoxes();
}

// Draw just one reload slot (label, wire dot if applicable, and its button).
// Used both for the full page and for in-place refresh after a Load press,
// so tapping a button doesn't force a full-screen redraw (which flickers).
void drawReloadButton(int i) {
  int labelY = partLabelY(i);
  int btnX = partReloadBtnX(i);
  int buttonY = partButtonY(i);
  const char* labelText = (partType[i] == TYPE_WIRE) ? wireLoadLabel[i] : parts[i];
  drawCenteredText(btnX, labelY, RELOAD_BTN_W, LABEL_H, labelText, RA8875_WHITE, RA8875_BLACK, 1);

  if (partType[i] == TYPE_WIRE) {
    int dotX = btnX + RELOAD_BTN_W - WIRE_DOT_MARGIN;
    int dotY = labelY + LABEL_H / 2;
    uint16_t dotColor = (wireStock[i] == WIRE_INSTOCK) ? COLOR_GREEN : COLOR_RED;
    tft.fillCircle(dotX, dotY, WIRE_DOT_RADIUS, dotColor);
    tft.drawCircle(dotX, dotY, WIRE_DOT_RADIUS, RA8875_WHITE);
  }

  uint16_t fillColor = COLOR_PLUS;
  const char* label = "LOAD";
  uint8_t labelSize = 1;
  if (partType[i] != TYPE_WIRE) {
    switch (dispenserFlag[i]) {
      case FLAG_EMPTY:
        fillColor = COLOR_PLUS;
        label = "LOAD";
        labelSize = 1;
        break;
      case FLAG_LOADED:
        fillColor = COLOR_LOADED;
        label = "LOADED";
        labelSize = 1;
        break;
      case FLAG_REMOVE:
        fillColor = COLOR_RED;
        label = "UNABLE TO LOAD";
        labelSize = 0;
        break;
    }
  }
  drawButton(btnX, buttonY, RELOAD_BTN_W, RELOAD_BTN_H, fillColor, RA8875_WHITE, label, labelSize);
}

void drawReloadPage() {
  for (int i = 0; i < NUM_PARTS; i++) {
    drawReloadButton(i);
  }

  tft.fillRect(legendX, legendY, legendW, legendH, RA8875_DARKGREY);
  drawCenteredText(legendX, legendY + 5, legendW, 30, "Wire Status", RA8875_WHITE, RA8875_DARKGREY, 1);
  tft.fillCircle(legendDotX, legendRow1Y, WIRE_DOT_RADIUS, COLOR_GREEN);
  tft.drawCircle(legendDotX, legendRow1Y, WIRE_DOT_RADIUS, RA8875_WHITE);
  drawLeftText(legendTextX, legendRow1Y, "In Stock", RA8875_WHITE, RA8875_DARKGREY, 1);
  tft.fillCircle(legendDotX, legendRow2Y, WIRE_DOT_RADIUS, COLOR_RED);
  tft.drawCircle(legendDotX, legendRow2Y, WIRE_DOT_RADIUS, RA8875_WHITE);
  drawLeftText(legendTextX, legendRow2Y, "Empty", RA8875_WHITE, RA8875_DARKGREY, 1);

  drawButtonTwoLine(reloadAllBtnX, reloadAllBtnY, reloadAllBtnW, reloadAllBtnH,
                    COLOR_PLUS, RA8875_WHITE, "LOAD RESISTORS", "AND CAPACITORS", 1);
}

void drawInterface() {
  tft.fillScreen(RA8875_BLACK);
  drawTabBar();
  if (currentPage == PAGE_DISPENSE) drawDispensePage();
  else drawReloadPage();
}

// Non-closeable popup shown while runDispense() is running, same size/position
// as the warning popup box (overlays the menu rather than covering the whole
// screen). It has no close button; since runDispense() blocks the loop() call
// that invoked it, the touch handler never runs again until dispensing is
// finished, so no touches can reach any control while this is up.
void showDispensingPopup() {
  dispensing = true;
  tft.fillRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_WHITE);
  tft.drawRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_BLACK);
  drawCenteredText(popupX + 10, popupY + 10, POPUP_W - 20, POPUP_H - 20,
                   "Dispensing in progress...", RA8875_BLACK, RA8875_WHITE, 1);
}

void showWarningPopup(const char* message) {
  popupVisible = true;
  tft.fillRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_WHITE);
  tft.drawRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_BLACK);
  drawCenteredText(popupX + 10, popupY + 10, POPUP_W - 20, POPUP_H - 20, message, RA8875_BLACK, RA8875_WHITE, 1);
  drawCenteredText(popupCloseX, popupCloseY, POPUP_CLOSE_SIZE, POPUP_CLOSE_SIZE, "X", COLOR_RED, RA8875_WHITE, 1);
}
void closeWarningPopup() {
  popupVisible = false;
  drawInterface();
}

// =================================================================
//  INIT
// =================================================================
void initSR(byte ser, byte rclk, byte srclk, byte& state) {
  pinMode(ser, OUTPUT);
  pinMode(rclk, OUTPUT);
  pinMode(srclk, OUTPUT);
  state = 0xFF;
  shiftOut595(ser, rclk, srclk, state);
}
void initMotorPins(byte dirPin, byte stepPin) {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);
}

void setup() {
  Serial.begin(9600);

  // Shift registers
  initSR(SRRC_SER, SRRC_RCLK, SRRC_SRCLK, srRC);
  initSR(SRW_SER, SRW_RCLK, SRW_SRCLK, srW);

  // Capacitor + resistor motor pins & sensors
  for (int s = 0; s < 3; s++) {
    initMotorPins(cap[s].dirPin, cap[s].stepPin);
    pinMode(cap[s].sensorPin, INPUT);
    cap[s].cutter.attach(cap[s].servoPin, 500, 2500);
    cap[s].cutter.write(CUT_REST_ANGLE);

    initMotorPins(res[s].dirPin, res[s].stepPin);
    pinMode(res[s].sensorPin, INPUT);
  }

  // Wire cutter motor pins & sensors
  initMotorPins(WM1_DIR, WM1_STEP);
  initMotorPins(WM2_DIR, WM2_STEP);
  for (int s = 0; s < 3; s++) {
    initMotorPins(wire[s].feedDir, wire[s].feedStep);
    pinMode(wire[s].sensorPin, INPUT);
  }
  pinMode(SW_PIN, INPUT_PULLUP);
  swLastState = digitalRead(SW_PIN);

  //Wire guillotine aligns
  homeM1M2();

  // Packing system motor + limit switch (enable is on SRW bit 5)
  initMotorPins(PACK_DIR, PACK_STEP);
  pinMode(PACK_LIMIT_PIN, INPUT_PULLUP);

  // Door sensors
  pinMode(DOOR_LEFT_PIN, INPUT_PULLUP);
  pinMode(DOOR_RIGHT_PIN, INPUT_PULLUP);
  leftDoorState = readDoorOpen(DOOR_LEFT_PIN) ? DOOR_OPEN : DOOR_CLOSED;
  rightDoorState = readDoorOpen(DOOR_RIGHT_PIN) ? DOOR_OPEN : DOOR_CLOSED;

  // PCA9685 (resistor cutters)
  i2cRecover();
  Wire.begin();
  Wire.setClock(100000);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  for (int s = 0; s < 3; s++) {
    moveServoCh(res[s].cutChA, RCUT_A_REST);
    moveServoCh(res[s].cutChB, RCUT_B_REST);
  }
  delay(RCUT_TRAVEL_MS);

  // Seed status flags from sensors
  for (int s = 0; s < 3; s++) {
    bool capPresent = schmitt(cap[s].sensorPin, cap[s].blocked,
                              cap[s].hi, cap[s].lo);
    bool resPresent = schmitt(res[s].sensorPin, res[s].blocked,
                              res[s].hi, res[s].lo);

    dispenserFlag[3 + s] = capPresent ? FLAG_LOADED : FLAG_EMPTY;
    dispenserFlag[6 + s] = resPresent ? FLAG_LOADED : FLAG_EMPTY;
    res[s].needExtra = resPresent;
    wireUpdateStock(s);
  }

  // Display
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 not found");
    while (1)
      ;
  }
  tft.displayOn(true);
  tft.GPIOX(true);
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
  tft.PWM1out(255);
  tft.touchEnable(true);

  drawInterface();
}

// =================================================================
//  LOOP / TOUCH HANDLER
// =================================================================
void handleSwitch() {
  int state = digitalRead(SW_PIN);
  if (state != swLastState) {
    delay(SW_DEBOUNCE_MS);
    state = digitalRead(SW_PIN);
    if (state != swLastState) swLastState = state;
  }
}

void loop() {
  if (dispensing) return;  // belt-and-suspenders: block all touch handling while dispensing

  handleSwitch();
  updateLoadStatusFromSensors();

  uint16_t rawX, rawY;
  int x, y;
  static unsigned long lastTouchTime = 0;

  if (!tft.touched()) return;
  if (!tft.touchRead(&rawX, &rawY)) return;
  if (millis() - lastTouchTime < 300) return;
  lastTouchTime = millis();

  touchToScreen(rawX, rawY, x, y);

  if (popupVisible) {
    if (pointInRect(x, y, popupCloseX, popupCloseY, POPUP_CLOSE_SIZE, POPUP_CLOSE_SIZE))
      closeWarningPopup();
    return;
  }

  // Tab bar
  if (pointInRect(x, y, 0, 0, SCREEN_WIDTH / 2, TAB_H)) {
    if (currentPage != PAGE_DISPENSE) {
      currentPage = PAGE_DISPENSE;
      drawInterface();
    }
    return;
  }
  if (pointInRect(x, y, SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H)) {
    if (currentPage != PAGE_RELOAD) {
      currentPage = PAGE_RELOAD;
      drawInterface();
    }
    return;
  }

  // Reload page
  if (currentPage == PAGE_RELOAD) {
    for (int i = 0; i < NUM_PARTS; i++) {
      if (pointInRect(x, y, partReloadBtnX(i), partButtonY(i), RELOAD_BTN_W, RELOAD_BTN_H)) {
        if (partType[i] == TYPE_WIRE) {
          reloadPart(i);
        } else if (dispenserFlag[i] == FLAG_EMPTY) {
          reloadPart(i);
        } else if (dispenserFlag[i] == FLAG_REMOVE) {
          removeExtra(i);
        }
        // Ignore the touch that is still held after the (long) load routine:
        while (tft.touched()) { tft.touchRead(&rawX, &rawY); }  // wait for release
        lastTouchTime = millis();                               // reset debounce
        drawReloadButton(i);                                    // refresh just this slot — no full-screen redraw
        return;
      }
    }
    if (pointInRect(x, y, reloadAllBtnX, reloadAllBtnY, reloadAllBtnW, reloadAllBtnH)) {
      loadResistorsAndCapacitors();
      // Ignore the touch that is still held after the (long) batch:
      while (tft.touched()) { tft.touchRead(&rawX, &rawY); }  // wait for release
      lastTouchTime = millis();                               // reset debounce
      // only R/C slots can change; repaint those in place
      for (int i = 3; i < NUM_PARTS; i++) drawReloadButton(i);
      return;
    }
    return;
  }

  // Component +/- buttons
  for (int i = 0; i < NUM_PARTS; i++) {
    int buttonY = partButtonY(i);
    if (pointInRect(x, y, partMinusX(i), buttonY, BTN_W, BTN_H)) {
      if (partEnabled[i] && quantities[i] > 0) {
        quantities[i]--;
        redrawQuantity(i);
      }
      return;
    }
    if (pointInRect(x, y, partPlusX(i), buttonY, BTN_W, BTN_H)) {
      if (partEnabled[i] && quantities[i] < 9) {
        quantities[i]++;
        redrawQuantity(i);
      }
      return;
    }
  }

  // # of kits
  if (pointInRect(x, y, kitMinusX, kitButtonY, KIT_BTN_W, KIT_BTN_H)) {
    if (numKits > 0) {
      numKits--;
      redrawKitQuantity();
    }
    return;
  }
  if (pointInRect(x, y, kitPlusX, kitButtonY, KIT_BTN_W, KIT_BTN_H)) {
    if (numKits < 10) {
      numKits++;
      redrawKitQuantity();
    }
    return;
  }

  // DISPENSE
  if (pointInRect(x, y, dispenseBtnX, dispenseBtnY, dispenseBtnW, dispenseBtnH)) {
    if (leftDoorState == DOOR_OPEN || rightDoorState == DOOR_OPEN) {
      showWarningPopup("Doors are open!");
      return;
    }
    if (numKits <= 0) {
      showWarningPopup("Set # of Kits first");
      return;
    }
    if (!selectedDispensersReady()) {
      showWarningPopup("Selected part not loaded");
      return;
    }
    showDispensingPopup();
    runDispense();
    dispensing = false;
    // Ignore the touch that is still held after the (long) batch:
    while (tft.touched()) { tft.touchRead(&rawX, &rawY); }  // wait for release
    lastTouchTime = millis();                               // reset debounce
    showWarningPopup("Dispensing complete!");
    return;
  }
}