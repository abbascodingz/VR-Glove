#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>

class ComplementaryFilter {
public:
    ComplementaryFilter(); // Constructor
    void begin();
    void update();
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }

private:
    Adafruit_MPU6050 mpu;
    unsigned long lastTime;
    float deltaTime;
    float roll, pitch, yaw;

    // Accelerometer and gyroscope raw values
    float ax, ay, az;
    int16_t gx, gy, gz;

    void readMPU6050Data();
};

#endif // COMPLEMENTARY_FILTER_H
