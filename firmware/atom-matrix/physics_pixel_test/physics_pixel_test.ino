#include <M5Unified.h>

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    
    // Explicitly initialize the physical IMU sensor
    M5.Imu.init(); 

    // Initialize the Serial hardware port
    Serial.begin(115200);
    delay(500); 
    Serial.println("Atom Matrix IMU Monitor - Starting Telemetry Stream...");
}

void loop() {
    // Keep internal hardware registers refreshed
    M5.update();

    // Check if the IMU has an active data chunk ready
    auto imu_update = M5.Imu.update();
    if (imu_update) {
        auto imuData = M5.Imu.getImuData();

        // Extracted raw force vector floats (G-force metrics)
        float accX = -imuData.accel.x; 
        float accY = imuData.accel.y;
        float accZ = imuData.accel.z;

        // Print values clean and fast
        Serial.printf("X: %.3f | Y: %.3f | Z: %.3f\n", accX, accY, accZ);
    }

    delay(20); // Steady 50Hz data collection rate
}
