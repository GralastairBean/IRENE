#include <Servo.h>

// --- Configuration ---
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

const int ANGLE_MIN  = 0;       // raise to 10 to be gentle on gears
const int ANGLE_MAX  = 180;     // lower to 170 to be gentle on gears

const int MIN_HOLD_MS = 150;    // shortest time before picking new target
const int MAX_HOLD_MS = 800;    // longest time before picking new target

// --- Per-servo state ---
Servo s1, s2;
unsigned long s1_nextChange = 0;
unsigned long s2_nextChange = 0;

void pickNewTarget(Servo& servo, unsigned long& nextChange) {
  int target = random(ANGLE_MIN, ANGLE_MAX + 1);
  servo.write(target);
  nextChange = millis() + random(MIN_HOLD_MS, MAX_HOLD_MS + 1);
}

void setup() {
  randomSeed(analogRead(A0));   // floating pin = noise = different seed each boot

  s1.attach(SERVO1_PIN);
  s2.attach(SERVO2_PIN);
  s1.write(90);
  s2.write(90);
  delay(500);                   // let them settle at center

  // Trigger immediate first random move
  s1_nextChange = millis();
  s2_nextChange = millis();
}

void loop() {
  if (millis() >= s1_nextChange) pickNewTarget(s1, s1_nextChange);
  if (millis() >= s2_nextChange) pickNewTarget(s2, s2_nextChange);
}