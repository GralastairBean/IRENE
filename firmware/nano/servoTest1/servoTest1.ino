#include <Servo.h>

// --- Configuration ---
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int STEP_DEG   = 2;       // degrees per sweep step

// --- Per-servo state ---
struct ServoState {
  Servo    servo;
  int      pin;
  int      angle;
  int      direction;       // 1 = up, -1 = down
  bool     sweeping;
  int      stepDelay;       // ms between sweep steps
  unsigned long lastStep;
  const char* name;
};

ServoState s1 = { Servo(), SERVO1_PIN, 90, 1, false, 20, 0, "Servo 1" };
ServoState s2 = { Servo(), SERVO2_PIN, 90, 1, false, 20, 0, "Servo 2" };

ServoState* active = &s1;     // currently selected servo

void printHelp() {
  Serial.println(F("=== Dual Servo Control ==="));
  Serial.println(F("  1      : select Servo 1 (pin 9)"));
  Serial.println(F("  2      : select Servo 2 (pin 10)"));
  Serial.println(F("  s      : start sweep on active servo"));
  Serial.println(F("  x      : stop sweep on active servo"));
  Serial.println(F("  S      : start sweep on BOTH servos"));
  Serial.println(F("  X      : stop sweep on BOTH servos"));
  Serial.println(F("  c      : center active servo to 90"));
  Serial.println(F("  C      : center BOTH servos to 90"));
  Serial.println(F("  +      : speed up active servo"));
  Serial.println(F("  -      : slow down active servo"));
  Serial.println(F("  <num>  : move active servo to angle (0-180)"));
  Serial.println(F("  ?      : print status"));
  Serial.println(F("  h      : show this help"));
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("Active: ")); Serial.println(active->name);
  Serial.print(F("S1: angle=")); Serial.print(s1.angle);
  Serial.print(F(" sweep=")); Serial.print(s1.sweeping ? "ON " : "off");
  Serial.print(F(" delay=")); Serial.print(s1.stepDelay); Serial.println(F("ms"));
  Serial.print(F("S2: angle=")); Serial.print(s2.angle);
  Serial.print(F(" sweep=")); Serial.print(s2.sweeping ? "ON " : "off");
  Serial.print(F(" delay=")); Serial.print(s2.stepDelay); Serial.println(F("ms"));
}

void setup() {
  Serial.begin(115200);
  s1.servo.attach(s1.pin);
  s2.servo.attach(s2.pin);
  s1.servo.write(s1.angle);
  s2.servo.write(s2.angle);
  printHelp();
  Serial.println(F("Ready. Active = Servo 1. Both sweeps STOPPED."));
}

void handleCommand(String input) {
  if (input.length() == 0) return;

  // Servo selection
  if (input == "1") {
    active = &s1;
    Serial.println(F(">> Active: Servo 1"));
    return;
  }
  if (input == "2") {
    active = &s2;
    Serial.println(F(">> Active: Servo 2"));
    return;
  }

  // Single-servo commands (operate on active)
  if (input == "s") {
    active->sweeping = true;
    Serial.print(F(">> ")); Serial.print(active->name); Serial.println(F(" sweep STARTED"));
    return;
  }
  if (input == "x") {
    active->sweeping = false;
    Serial.print(F(">> ")); Serial.print(active->name);
    Serial.print(F(" sweep STOPPED at ")); Serial.println(active->angle);
    return;
  }
  if (input == "c") {
    active->sweeping = false;
    active->angle = 90;
    active->servo.write(90);
    Serial.print(F(">> ")); Serial.print(active->name); Serial.println(F(" centered"));
    return;
  }
  if (input == "+") {
    active->stepDelay = max(2, active->stepDelay - 5);
    Serial.print(F(">> ")); Serial.print(active->name);
    Serial.print(F(" step delay: ")); Serial.print(active->stepDelay); Serial.println(F("ms"));
    return;
  }
  if (input == "-") {
    active->stepDelay = min(200, active->stepDelay + 5);
    Serial.print(F(">> ")); Serial.print(active->name);
    Serial.print(F(" step delay: ")); Serial.print(active->stepDelay); Serial.println(F("ms"));
    return;
  }

  // Both-servo commands (capitalized)
  if (input == "S") {
    s1.sweeping = true;
    s2.sweeping = true;
    Serial.println(F(">> BOTH sweeps STARTED"));
    return;
  }
  if (input == "X") {
    s1.sweeping = false;
    s2.sweeping = false;
    Serial.println(F(">> BOTH sweeps STOPPED"));
    return;
  }
  if (input == "C") {
    s1.sweeping = false; s1.angle = 90; s1.servo.write(90);
    s2.sweeping = false; s2.angle = 90; s2.servo.write(90);
    Serial.println(F(">> BOTH centered"));
    return;
  }

  // Help / status
  if (input == "h") { printHelp(); return; }
  if (input == "?") { printStatus(); return; }

  // Numeric: move active servo to angle
  bool looksNumeric = (input.charAt(0) == '-' || isDigit(input.charAt(0)));
  if (looksNumeric) {
    int target = input.toInt();
    if (target >= 0 && target <= 180) {
      active->sweeping = false;
      active->angle = target;
      active->servo.write(target);
      Serial.print(F(">> ")); Serial.print(active->name);
      Serial.print(F(" moved to ")); Serial.println(target);
      return;
    }
  }

  Serial.print(F("?? Unknown command: "));
  Serial.println(input);
}

void updateSweep(ServoState& s) {
  if (!s.sweeping) return;
  if (millis() - s.lastStep < (unsigned long)s.stepDelay) return;

  s.lastStep = millis();
  s.angle += STEP_DEG * s.direction;
  if (s.angle >= 180) { s.angle = 180; s.direction = -1; }
  if (s.angle <= 0)   { s.angle = 0;   s.direction =  1; }
  s.servo.write(s.angle);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleCommand(input);
  }

  updateSweep(s1);
  updateSweep(s2);
}