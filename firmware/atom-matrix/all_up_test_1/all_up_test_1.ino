#include <Arduino.h>
#include <M5Unified.h>    // Core architecture + Bosch BMI270 sensor handling
#include <FastLED.h>      // Dedicated hardware library for the NeoPixel matrix
#include <ESP32Servo.h>   // Required hardware timer servo replacement for ESP32
#include <math.h>

// =================== MATRIX DISPLAY CONFIG ===================
#define NUM_LEDS 25
#define DATA_PIN 27
CRGB leds[NUM_LEDS];      // FastLED memory buffer allocation
const int LCDbrightness = 20;

// =================== SERVO CONFIG ===================
const int SERVO1_PIN = 26; 
const int SERVO2_PIN = 32; 

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 180;

const float S1_DEG_PER_SEC = 150.0;
const float S2_DEG_PER_SEC = 150.0;

const int MIN_HOLD_MS = 200;
const int MAX_HOLD_MS = 1000;
const int UPDATE_MS = 20;

const int REST_ANGLE = 90;

// =================== TILT CONFIG ===================
const float ALPHA = 0.85; 
const float VERTICAL_THRESHOLD_DEG = 20.0;
const float RESUME_THRESHOLD_DEG = 30.0;

const int STABLE_TIME_MS = 100; 
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
                   STATUS_MOVING,
                   STATUS_OFF }; 

ServoState s1 = { Servo(), SERVO1_PIN, 90.0, 90, 0, 0, true };
ServoState s2 = { Servo(), SERVO2_PIN, 90.0, 90, 0, 0, true };

bool imuOk = false;
bool systemOff = false; 

float filteredX = 0.0;
float filteredY = 0.0;
float filteredZ = 1.0;

float fusedTilt = 0.0;
unsigned long lastImuMicros = 0;
unsigned long lastImuPollMs = 0;
unsigned long lastUpdate = 0;
unsigned long verticalSince = 0;
unsigned long resumeAt = 0;

bool isVertical = false;

// =================== IMU INITIALIZATION ===================
void initIMU() {
  if (M5.Imu.isEnabled()) {
    imuOk = true;
  } else {
    imuOk = false;
    Serial.println("CRITICAL ERROR: Onboard BMI270 IMU not detected!");
    return;
  }

  M5.Imu.update();
  auto imu_data = M5.Imu.getImuData();
  
  filteredX = imu_data.accel.x;
  filteredY = imu_data.accel.y;
  filteredZ = imu_data.accel.z;
  
  lastImuMicros = micros();
}

// =================== DIRECT VECTOR ACCELERATION MAPPING ===================
void updateTilt() {
  if (!imuOk || systemOff) return; 

  M5.Imu.update();
  auto imu_data = M5.Imu.getImuData();

  unsigned long now = micros();
  float dt = (now - lastImuMicros) / 1000000.0;
  lastImuMicros = now;
  if (dt <= 0 || dt > 0.5) dt = 0.01;

  filteredX = ALPHA * (filteredX + (imu_data.gyro.y * M_PI / 180.0 * filteredZ - imu_data.gyro.z * M_PI / 180.0 * filteredY) * dt) + (1.0 - ALPHA) * imu_data.accel.x;
  filteredY = ALPHA * (filteredY + (imu_data.gyro.z * M_PI / 180.0 * filteredX - imu_data.gyro.x * M_PI / 180.0 * filteredZ) * dt) + (1.0 - ALPHA) * imu_data.accel.y;
  filteredZ = ALPHA * (filteredZ + (imu_data.gyro.x * M_PI / 180.0 * filteredY - imu_data.gyro.y * M_PI / 180.0 * filteredX) * dt) + (1.0 - ALPHA) * imu_data.accel.z;

  float norm = sqrt(filteredX * filteredX + filteredY * filteredY + filteredZ * filteredZ);
  if (norm > 0) {
    filteredX /= norm;
    filteredY /= norm;
    filteredZ /= norm;
  }

  float targetAxisG = filteredZ; 
  fusedTilt = acos(constrain(fabs(targetAxisG), -1.0, 1.0)) * 180.0 / M_PI;

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
  if (!s.servo.attached()) return; // Protect execution loops if hardware is detached
  
  float diff = target - s.currentAngle;
  if (fabs(diff) <= s.degPerStep) {
    s.currentAngle = target;
  } else {
    s.currentAngle += (diff > 0 ? s.degPerStep : -s.degPerStep);
  }
  s.servo.write((int)(s.currentAngle + 0.5));
}

void updateMotion(ServoState& s) {
  // If the system is toggled off, exit tracking cycles entirely (handled by attach/detach toggles now)
  if (systemOff) return;

  if (isVertical || (millis() < resumeAt)) {
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
  
  if (s.servo.attached()) {
    s.servo.write((int)(s.currentAngle + 0.5));
  }
}

// Status lights controller mapping
void updateMatrixLEDs(RobotStatus status) {
  CRGB statusColor = CRGB::Black;
  
  switch (status) {
    case STATUS_NO_IMU:   statusColor = CRGB::Purple; break; 
    case STATUS_VERTICAL: statusColor = CRGB::Green;  break; 
    case STATUS_PAUSED:   statusColor = CRGB::Yellow; break; 
    case STATUS_MOVING:   statusColor = CRGB::Red;    break; 
    case STATUS_OFF:      statusColor = CRGB::Black;  break; 
  }
  
  fill_solid(leds, NUM_LEDS, statusColor);
  FastLED.show();
}

// =================== MAIN INITIALIZATION ===================
void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LCDbrightness); 
  
  Serial.println("\n--- ATOM-MATRIX V1.1 CORE CONTROL ENGINE BOOTED ---");

  randomSeed(esp_random());
  initIMU();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  s1.servo.setPeriodHertz(50); 
  s2.servo.setPeriodHertz(50);

  s1.degPerStep = S1_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  s2.degPerStep = S2_DEG_PER_SEC * (UPDATE_MS / 1000.0);
  
  // Initial hardware configuration line bindings
  s1.servo.attach(s1.pin, 500, 2400); 
  s2.servo.attach(s2.pin, 500, 2400);
  
  s1.servo.write(REST_ANGLE);
  s2.servo.write(REST_ANGLE);
  s1.currentAngle = REST_ANGLE;
  s2.currentAngle = REST_ANGLE;
  delay(200);

  s1.holdUntil = millis();
  s2.holdUntil = millis();
  resumeAt = millis() + RESUME_DELAY_MS;
}

void loop() {
  M5.update(); 

  // Handle tactile display button clicks
  if (M5.BtnA.wasPressed()) {
    systemOff = !systemOff; 
    
    if (systemOff) {
      // UPGRADE: Sever PWM signal completely to safely drop motor energy load to 0%
      s1.servo.detach();
      s2.servo.detach();
    } else {
      // UPGRADE: Re-engage software control routing lines
      s1.servo.attach(s1.pin, 500, 2400);
      s2.servo.attach(s2.pin, 500, 2400);
      
      // Enforce clean state variables wipe down
      isVertical = true;                       
      verticalSince = millis();                
      resumeAt = millis() + RESUME_DELAY_MS;   
      
      s1.currentAngle = REST_ANGLE;
      s2.currentAngle = REST_ANGLE;
      s1.servo.write(REST_ANGLE);
      s2.servo.write(REST_ANGLE);
      s1.atTarget = true;
      s2.atTarget = true;
      s1.holdUntil = resumeAt;
      s2.holdUntil = resumeAt;
    }
    
    Serial.print("SYSTEM MODE TOGGLED. State: ");
    Serial.println(systemOff ? "OFF (DEPOWERED)" : "ACTIVE (RUN)");
  }

  if (systemOff) {
    updateMatrixLEDs(STATUS_OFF);
  }

  if (millis() - lastImuPollMs >= (unsigned long)IMU_POLL_MS) {
    lastImuPollMs = millis();
    updateTilt();
  }

  if (millis() - lastUpdate >= (unsigned long)UPDATE_MS) {
    lastUpdate = millis();
    updateMotion(s1);
    updateMotion(s2);
  }

  // ---- Status output (only on change or if status state switches) ----
  static RobotStatus lastStatusPrinted = (RobotStatus)-1;
  static float lastTiltPrinted = -999.0;
  const float TILT_CHANGE_DEG = 2.0;

  RobotStatus status;
  if (systemOff)       status = STATUS_OFF;
  else if (!imuOk)     status = STATUS_NO_IMU;
  else if (isVertical) status = STATUS_VERTICAL;
  else if (millis() < resumeAt) status = STATUS_PAUSED;
  else                 status = STATUS_MOVING;

  bool statusChanged = (status != lastStatusPrinted);
  bool tiltChanged = !systemOff && (fabs(fusedTilt - lastTiltPrinted) >= TILT_CHANGE_DEG);

  if (statusChanged || tiltChanged) {
    lastStatusPrinted = status;
    lastTiltPrinted = fusedTilt;
    
    if (!systemOff) {
      updateMatrixLEDs(status);
    }

    const char* label;
    switch (status) {
      case STATUS_OFF:      label = "SYSTEM OFF"; break;
      case STATUS_NO_IMU:   label = "NO IMU   "; break;
      case STATUS_VERTICAL: label = "UPRIGHT  "; break;
      case STATUS_PAUSED:   label = "WAIT     "; break;
      default:              label = "MOVING   "; break;
    }

    char bar[19]; 
    int bars = systemOff ? 0 : (int)(fusedTilt / 5.0);
    if (bars > 18) bars = 18;
    for (int i = 0; i < 18; i++) bar[i] = (i < bars) ? '#' : '.';
    bar[18] = '\0';

    Serial.print("[");
    Serial.print(label);
    Serial.print("]  True Deviation: ");
    if (systemOff) {
      Serial.println("LOCKED |--- SERVOS DEPOWERED ---|");
    } else {
      if (fusedTilt < 10) Serial.print(" ");
      if (fusedTilt < 100) Serial.print(" ");
      Serial.print(fusedTilt, 1);
      Serial.print(" deg  |");
      Serial.print(bar);
      Serial.println("|");
    }
  }
}
