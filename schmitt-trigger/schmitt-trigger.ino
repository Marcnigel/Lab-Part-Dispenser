// ================================================================
//  Photo-interrupter Schmitt trigger test + counter
//  - Prints raw value + state continuously
//  - Counts each resistor pass (CLEAR→BLOCKED→CLEAR)
//  - Type "end" in serial monitor to stop and print the total
// ================================================================

#define SENSOR_PIN   A4
#define SENSOR_HI    950   // rise above this → BLOCKED
#define SENSOR_LO    350   // fall below this → CLEAR

bool blocked  = false;   // debounced sensor state
bool prevBlocked = false; // previous state, for edge detection
long count    = 0;       // resistors counted so far
bool running  = true;    // false once user types "end"

// Two-threshold hysteresis. Updates 'state', returns new level.
bool schmitt(byte pin, bool& state, int hi, int lo) {
  int v = analogRead(pin);
  if (state) {                 // currently BLOCKED
    if (v < lo) state = false;
  } else {                     // currently CLEAR
    if (v > hi) state = true;
  }
  return state;
}

// Watches for a trailing edge (BLOCKED→CLEAR) and increments count.
// Returns true on the step where a new resistor was counted.
bool updateCount() {
  bool counted = false;
  if (prevBlocked && !blocked) {   // just went BLOCKED → CLEAR
    count++;
    counted = true;
  }
  prevBlocked = blocked;
  return counted;
}

// Checks serial for "end" command. Returns true if stop was requested.
bool checkForEnd() {
  if (!Serial.available()) return false;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  return (line == "end");
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  pinMode(SENSOR_PIN, INPUT);

  blocked = schmitt(SENSOR_PIN, blocked, SENSOR_HI, SENSOR_LO);
  prevBlocked = blocked;

  Serial.println("Photo-interrupter test. RAW | STATE | COUNT");
  Serial.println("Type 'end' to stop and see the total.");
}

void loop() {
  if (!running) return;

  if (checkForEnd()) {
    running = false;
    Serial.print("=== STOPPED. Total resistors counted: ");
    Serial.print(count);
    Serial.println(" ===");
    return;
  }

  int raw = analogRead(SENSOR_PIN);
  blocked = schmitt(SENSOR_PIN, blocked, SENSOR_HI, SENSOR_LO);
  updateCount();

  Serial.print("RAW: ");
  Serial.print(raw);
  Serial.print("\tSTATE: ");
  Serial.print(blocked ? "BLOCKED" : "CLEAR");
  Serial.print("\tCOUNT: ");
  Serial.println(count);

  delay(50);
}