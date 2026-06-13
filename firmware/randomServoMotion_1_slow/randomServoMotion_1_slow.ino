#include <Servo.h>

// --- Configuration ---
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

const int ANGLE_MIN  = 0;       // raise to 10 to be gentle on gears
const int ANGLE_MAX  = 180;     // lower to 170 to be gentle on gears

// Motion speed (degrees per second). Lower = smoother/slower.
const float S1_DEG_PER_SEC = 90.0;   // ~2 sec for full sweep
const float S2_DEG_PER_SEC = 70.0;

// How long to pause at each target before picking a new one (ms)
const int MIN_HOLD_MS = 200;
const int MAX_HOLD_MS = 1000;

const int UPDATE_MS = 20;       // motion update rate (50 Hz)

// --- Per-servo state ---
struct ServoState {
  Servo servo;
  int   pin;
  float currentAngle;     // float for smooth fractional interpolation
  int   targetAngle;
  float degPerStep;       // computed from deg/sec and UPDATE_MS
  unsigned long holdUntil;   // when we're allowed to pick new target
  bool  atTarget;
};

ServoState s1 = { Servo(), SERVO1_PIN, 90.0, 90, 0, 0, true };
ServoState s2 = { Servo(), SERVO2_PIN, 90.0, 90, 0, 0, true };

unsigned long lastUpdate = 0;

void pickNewTarget(ServoState& s) {
  s.targetAngle = random(ANGLE_MIN, ANGLE_MAX + 1);
  s.atTarget = false;
}

void updateMotion(ServoState& s) {
  if (s.atTarget) {
    // Currently holding at target — see if it's time to pick a new one
    if (millis() >= s.holdUntil) {
      pickNewTarget(s);
    }
    return;
  }

  // Move toward target by degPerStep
  float diff = s.targetAngle - s.currentAngle;
  if (fabs(diff) <= s.degPerStep) {
    // Close enough — snap to target and start holding
    s.currentAngle = s.targetAngle;
    s.atTarget = true;
    s.holdUntil = millis() + random(MIN_HOLD_MS, MAX_HOLD_MS + 1);
  } else {
    s.currentAngle += (diff > 0 ? s.degPerStep : -s.degPerStep);
  }

  s.servo.write((int)(s.currentAngle + 0.5));
}

void setup() {
  randomSeed(analogRead(A0));

  // Convert deg/sec to deg per update tick
  s1.degPerStep = S1_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  s2.degPerStep = S2_DEG_PER_SEC * (UPDATE_MS / 1000.0);

  s1.servo.attach(s1.pin);
  s2.servo.attach(s2.pin);
  s1.servo.write(90);
  s2.servo.write(90);
  delay(3500);

  // Kick off first random target immediately
  s1.holdUntil = millis();
  s2.holdUntil = millis();
}

void loop() {
  if (millis() - lastUpdate >= (unsigned long)UPDATE_MS) {
    lastUpdate = millis();
    updateMotion(s1);
    updateMotion(s2);
  }
}