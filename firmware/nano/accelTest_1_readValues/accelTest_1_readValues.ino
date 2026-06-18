#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =================== SERVO CONFIG ===================
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

const int ANGLE_MIN  = 5;
const int ANGLE_MAX  = 175;

const float S1_DEG_PER_SEC = 150.0;
const float S2_DEG_PER_SEC = 150.0;

const int MIN_HOLD_MS = 200;
const int MAX_HOLD_MS = 1000;
const int UPDATE_MS   = 20;

const int REST_ANGLE  = 90;     // where servos go when robot is upright

// =================== TILT CONFIG ===================
const float ALPHA = 0.98;       // complementary filter weight
const float VERTICAL_THRESHOLD_DEG = 20.0;   // tilt below this → upright
const float RESUME_THRESHOLD_DEG   = 30.0;   // must exceed this to resume
const int   STABLE_TIME_MS         = 400;    // must stay upright this long
const int   IMU_POLL_MS            = 10;     // 100 Hz

// =================== STATE ===================
struct ServoState {
  Servo servo;
  int   pin;
  float currentAngle;
  int   targetAngle;
  float degPerStep;
  unsigned long holdUntil;
  bool  atTarget;
};

ServoState s1 = { Servo(), SERVO1_PIN, 90.0, 90, 0, 0, true };
ServoState s2 = { Servo(), SERVO2_PIN, 90.0, 90, 0, 0, true };

Adafruit_MPU6050 mpu;
bool  imuOk = false;

float pitch = 0.0;
float roll  = 0.0;
float fusedTilt = 0.0;
unsigned long lastImuMicros = 0;
unsigned long lastImuPollMs = 0;
unsigned long lastUpdate    = 0;
unsigned long verticalSince = 0;

bool isVertical = false;        // confirmed-stable vertical state

// =================== IMU ===================
void initIMU() {
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND — running without tilt detection");
    imuOk = false;
    return;
  }
  imuOk = true;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Seed pitch/roll
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  pitch = atan2(a.acceleration.y,
                sqrt(a.acceleration.x*a.acceleration.x +
                     a.acceleration.z*a.acceleration.z)) * 180.0 / PI;
  roll  = atan2(-a.acceleration.x,
                sqrt(a.acceleration.y*a.acceleration.y +
                     a.acceleration.z*a.acceleration.z)) * 180.0 / PI;
  lastImuMicros = micros();

  Serial.println("MPU6050 ready");
}

void updateTilt() {
  if (!imuOk) return;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (now - lastImuMicros) / 1000000.0;
  lastImuMicros = now;
  if (dt <= 0 || dt > 0.5) dt = 0.01;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitchAccel = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
  float rollAccel  = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

  float gx_dps = g.gyro.x * 180.0 / PI;
  float gy_dps = g.gyro.y * 180.0 / PI;

  float pitchGyro = pitch + gx_dps * dt;
  float rollGyro  = roll  + gy_dps * dt;

  // Impact-aware blending
  float mag = sqrt(ax*ax + ay*ay + az*az);
  float gForce = mag / 9.81;
  float a_used = ALPHA;
  if (gForce < 0.7 || gForce > 1.3) a_used = 0.999;

  pitch = a_used * pitchGyro + (1.0 - a_used) * pitchAccel;
  roll  = a_used * rollGyro  + (1.0 - a_used) * rollAccel;

  float pr = pitch * PI / 180.0;
  float rr = roll  * PI / 180.0;
  fusedTilt = acos(constrain(cos(pr)*cos(rr), -1.0, 1.0)) * 180.0 / PI;

  // Hysteresis + stability timer
  if (fusedTilt < VERTICAL_THRESHOLD_DEG) {
    if (verticalSince == 0) verticalSince = millis();
    if (!isVertical && (millis() - verticalSince) > STABLE_TIME_MS) {
      isVertical = true;
      Serial.println(">>> UPRIGHT — pausing servos at 90/90");
    }
  } else if (fusedTilt > RESUME_THRESHOLD_DEG) {
    verticalSince = 0;
    if (isVertical) {
      isVertical = false;
      Serial.println(">>> Tilted — resuming random motion");
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
    // Smoothly slew to rest position and stay there
    slewTo(s, REST_ANGLE);
    s.atTarget = false;             // force re-pick when motion resumes
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

// =================== MAIN ===================
void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(analogRead(A0));

  initIMU();

  s1.degPerStep = S1_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  s2.degPerStep = S2_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  s1.servo.attach(s1.pin);
  s2.servo.attach(s2.pin);
  s1.servo.write(REST_ANGLE);
  s2.servo.write(REST_ANGLE);
  s1.currentAngle = REST_ANGLE;
  s2.currentAngle = REST_ANGLE;
  delay(500);

  s1.holdUntil = millis();
  s2.holdUntil = millis();
}

void loop() {
  if (millis() - lastImuPollMs >= (unsigned long)IMU_POLL_MS) {
    lastImuPollMs = millis();
    updateTilt();
  }

  if (millis() - lastUpdate >= (unsigned long)UPDATE_MS) {
    lastUpdate = millis();
    updateMotion(s1);
    updateMotion(s2);
  }

  // Optional: occasional status print
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    Serial.print("Tilt: "); Serial.print(fusedTilt, 1);
    Serial.print("°  ");
    Serial.println(isVertical ? "[UPRIGHT - paused]" : "[moving]");
  }
}