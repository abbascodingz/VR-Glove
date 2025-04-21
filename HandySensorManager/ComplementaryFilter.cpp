#include "ComplementaryFilter.h"
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// Define filter constant (typically 0.40 for slow hand movements)
#define ALPHA 0.40

ComplementaryFilter::ComplementaryFilter() : roll(0), pitch(0), yaw(0), lastTime(0), deltaTime(0) {}

void ComplementaryFilter::begin() {
    Wire.begin();
    mpu.begin();

    if (!mpu.begin()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
    Serial.println("MPU6050 initialized");
    lastTime = millis();
}

void ComplementaryFilter::readMPU6050Data() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate deltaTime (time difference between updates)
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
    lastTime = currentTime;

    // Calculate the roll and pitch angles using accelerometer data
    float accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    // Integrate gyroscope data to get angles (angular velocity * deltaTime)
    float gyroRoll = roll + g.gyro.x * deltaTime / 131.0; // 131 is the sensitivity for ±250°/s
    float gyroPitch = pitch + g.gyro.y * deltaTime / 131.0;
    float gyroYaw = yaw + g.gyro.z * deltaTime / 131.0;

    // Apply complementary filter
    roll = ALPHA * (gyroRoll)+(1 - ALPHA) * accelRoll;
    pitch = ALPHA * (gyroPitch)+(1 - ALPHA) * accelPitch;
    yaw = gyroYaw; // Using gyroscope data for yaw as accelerometer doesn't provide reliable yaw data
}

void ComplementaryFilter::update() {
    readMPU6050Data();
}
