#include <M5AtomS3.h>      // Core library for Atom-Matrix v1.1
#include <ESP32Servo.h>    // Required replacement for Servo.h on ESP32 boards
#include <math.h>

// =================== SERVO CONFIG ===================
// Remapped pins to physical Atom-Matrix side structural headers
const int SERVO1_PIN = 19; 
const int SERVO2_PIN = 22; 

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 160;

const float S1_DEG_PER_SEC = 150.0;
const float S2_DEG_PER_SEC = 150.0;

const int MIN_HOLD_MS = 200;
const int MAX_HOLD_MS = 1000;
const int UPDATE_MS = 20;

const int REST_ANGLE = 90;

// =================== TILT CONFIG ===================
const float ALPHA = 0.98;
const float VERTICAL_THRESHOLD_DEG = 20.0;
const float RESUME_THRESHOLD_DEG = 30.0;
const int STABLE_TIME_MS = 400;
const int RESUME_DELAY_MS = 1500;
const int IMU_POLL_MS = 10;

// =================== STATE ===================
struct ServoState {
  Servo servo;
  int pin;
  float currentAngle;
  int targetAngle;
  float degPerStep;
  unsigned long holdUntil;
  bool atTarget;
};

enum RobotStatus { STATUS_NO_IMU,
                   STATUS_VERTICAL,
                   STATUS_PAUSED,
                   STATUS_MOVING };

ServoState s1 = { Servo(), SERVO1_PIN, 90.0, 90, 0, 0, true };
ServoState s2 = { Servo(), SERVO2_PIN, 90.0, 90, 0, 0, true };

bool imuOk = false;

float pitch = 0.0;
float roll = 0.0;
float fusedTilt = 0.0;
unsigned long lastImuMicros = 0;
unsigned long lastImuPollMs = 0;
unsigned long lastUpdate = 0;
unsigned long verticalSince = 0;
unsigned long resumeAt = 0;

bool isVertical = false;

// =================== IMU (BOSCH BMI270) ===================
void initIMU() {
  // M5.begin() implicitly initializes the BMI270 IMU inside M5.Imu
  if (M5.Imu.begin()) {
    imuOk = true;
  } else {
    imuOk = false;
    return;
  }

  // Poll once to establish baseline orientation
  M5.Imu.update();
  float ax, ay, az;
  M5.Imu.getAccel(&ax, &ay, &az); // Values read in G-force units
  
  // Convert units to m/s^2 to maintain math compatibility with original code
  ax *= 9.81;
  ay *= 9.81;
  az *= 9.81;

  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  lastImuMicros = micros();
}

void updateTilt() {
  if (!imuOk) return;

  M5.Imu.update();
  float ax, ay, az;
  float gx, gy, gz;
  
  M5.Imu.getAccel(&ax, &ay, &az); // Returns Gs
  M5.Imu.getGyro(&gx, &gy, &gz);   // Returns Degrees Per Second (DPS)

  // Scale G values back up to m/s^2 for the existing algorithm
  ax *= 9.81;
  ay *= 9.81;
  az *= 9.81;

  unsigned long now = micros();
  float dt = (now - lastImuMicros) / 1000000.0;
  lastImuMicros = now;
  if (dt <= 0 || dt > 0.5) dt = 0.01;

  float pitchAccel = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  float rollAccel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  // BMI270 library handles DPS conversion internally; no radian translation required
  float gx_dps = gx;
  float gy_dps = gy;

  float pitchGyro = pitch + gx_dps * dt;
  float rollGyro = roll + gy_dps * dt;

  float mag = sqrt(ax * ax + ay * ay + az * az);
  float gForce = mag / 9.81;
  float a_used = ALPHA;
  if (gForce < 0.7 || gForce > 1.3) a_used = 0.999;

  pitch = a_used * pitchGyro + (1.0 - a_used) * pitchAccel;
  roll = a_used * rollGyro + (1.0 - a_used) * rollAccel;

  float pr = pitch * M_PI / 180.0;
  float rr = roll * M_PI / 180.0;
  fusedTilt = acos(constrain(cos(pr) * cos(rr), -1.0, 1.0)) * 180.0 / M_PI;

  if (fusedTilt < VERTICAL_THRESHOLD_DEG) {
    if (verticalSince == 0) verticalSince = millis();
    if (!isVertical && (millis() - verticalSince) > STABLE_TIME_MS) {
      isVertical = true;
    }
  } else if (fusedTilt > RESUME_THRESHOLD_DEG) {
    verticalSince = 0;
    if (isVertical) {
      isVertical = false;
      resumeAt = millis() + RESUME_DELAY_MS;
    }
  }
}

// =================== SERVO MOTION ===================
void pickNewTarget(ServoState& s) {
  s.targetAngle = random(ANGLE_MIN, ANGLE_MAX + 1);
  s.atTarget = false;
}

void slewTo(ServoState& s, int target) {
  float diff = target - s.currentAngle;
  if (fabs(diff) <= s.degPerStep) {
    s.currentAngle = target;
  } else {
    s.currentAngle += (diff > 0 ? s.degPerStep : -s.degPerStep);
  }
  s.servo.write((int)(s.currentAngle + 0.5));
}

void updateMotion(ServoState& s) {
  if (isVertical) {
    slewTo(s, REST_ANGLE);
    s.atTarget = false;
    s.holdUntil = millis();
    return;
  }

  if (millis() < resumeAt) {
    slewTo(s, REST_ANGLE);
    s.atTarget = false;
    s.holdUntil = millis();
    return;
  }

  if (s.atTarget) {
    if (millis() >= s.holdUntil) pickNewTarget(s);
    return;
  }

  float diff = s.targetAngle - s.currentAngle;
  if (fabs(diff) <= s.degPerStep) {
    s.currentAngle = s.targetAngle;
    s.atTarget = true;
    s.holdUntil = millis() + random(MIN_HOLD_MS, MAX_HOLD_MS + 1);
  } else {
    s.currentAngle += (diff > 0 ? s.degPerStep : -s.degPerStep);
  }
  s.servo.write((int)(s.currentAngle + 0.5));
}

// Helper to paint the built-in Matrix display to display statuses visually
void updateMatrixLEDs(RobotStatus status) {
  uint32_t color = 0x000000; // Off
  switch (status) {
    case STATUS_NO_IMU:   color = 0xFF0000; break; // Red
    case STATUS_VERTICAL: color = 0x00FF00; break; // Green
    case STATUS_PAUSED:   color = 0xFFFF00; break; // Yellow
    case STATUS_MOVING:   color = 0x0000FF; break; // Blue
  }
  M5.Disx.fillScreen(color);
}

// =================== MAIN ===================
void setup() {
  // Initialize the Atom-Matrix hardware subsystems safely
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Set LED matrix safely low to prevent localized heat building up
  M5.Disx.setBrightness(15); 
  
  // Seed using ESP32 system true-entropy hardware register
  randomSeed(esp_random());

  initIMU();

  // Standard ESP32Servo setup logic
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  s1.servo.setPeriodHertz(50); 
  s2.servo.setPeriodHertz(50);

  s1.degPerStep = S1_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  s2.degPerStep = S2_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  
  s1.servo.attach(s1.pin, 500, 2400); // Standard pulse width bounds
  s2.servo.attach(s2.pin, 500, 2400);
  
  s1.servo.write(REST_ANGLE);
  s2.servo.write(REST_ANGLE);
  s1.currentAngle = REST_ANGLE;
  s2.currentAngle = REST_ANGLE;
  delay(500);

  s1.holdUntil = millis();
  s2.holdUntil = millis();
  resumeAt = millis() + RESUME_DELAY_MS;
}

void loop() {
  M5.update(); // Maintain inner M5 hardware frames and button states

  if (millis() - lastImuPollMs >= (unsigned long)IMU_POLL_MS) {
    lastImuPollMs = millis();
    updateTilt();
  }

  if (millis() - lastUpdate >= (unsigned long)UPDATE_MS) {
    lastUpdate = millis();
    updateMotion(s1);
    updateMotion(s2);
  }

  // ---- Status output (only on change) ----
  static RobotStatus lastStatusPrinted = (RobotStatus)-1;
  static float lastTiltPrinted = -999.0;
  const float TILT_CHANGE_DEG = 2.0;

  RobotStatus status;
  if (!imuOk) status = STATUS_NO_IMU;
  else if (isVertical) status = STATUS_VERTICAL;
  else if (millis() < resumeAt) status = STATUS_PAUSED;
  else status = STATUS_MOVING;

  bool statusChanged = (status != lastStatusPrinted);
  bool tiltChanged = fabs(fusedTilt - lastTiltPrinted) >= TILT_CHANGE_DEG;

  if (statusChanged || tiltChanged) {
    lastStatusPrinted = status;
    lastTiltPrinted = fusedTilt;
    
    // Dynamically match onboard 5x5 array color to operating mode
    updateMatrixLEDs(status);

    const char* label;
    switch (status) {
      case STATUS_NO_IMU: label = "NO IMU "; break;
      case STATUS_VERTICAL: label = "UPRIGHT"; break;
      case STATUS_PAUSED: label = "WAIT   "; break;
      default: label = "MOVING "; break;
    }

    char bar[19];
    int bars = (int)(fusedTilt / 5.0);
    if (bars > 18) bars = 18;
    for (int i = 0; i < 18; i++) bar[i] = (i < bars) ? '#' : '.';
    bar[18] = '\0';

    // ESP32 directs primary logging out through standard Serial pipeline
    Serial.print("[");
    Serial.print(label);
    Serial.print("]  Tilt: ");
    if (fusedTilt < 10) Serial.print(" ");
    if (fusedTilt < 100) Serial.print(" ");
    Serial.print(fusedTilt, 1);
    Serial.print(" deg  |");
    Serial.print(bar);
    Serial.println("|");
  }
}
