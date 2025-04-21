#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>  // Only include necessary libraries for SensorManager
#include "FlexLibrary.h"  // Include FlexLibrary to use it
#include <Adafruit_MPU6050.h>
#include "ComplementaryFilter.h"  // Include ComplementaryFilter class

// SensorManager class definition
class SensorManager {
public:
    SensorManager(int thumbPin, int indexPin, int middlePin, int ringPin, int pinkyPin);

    void initializeSensors();
    void calibrateSensors();
    void readSensors();
    void printSensorData();
    void checkCalibrationButton();
    void readMPU6050Data();

private:
    // FlexLibrary objects for each finger
    Flex thumbFlex;
    Flex indexFlex;
    Flex middleFlex;
    Flex ringFlex;
    Flex pinkyFlex;

    ComplementaryFilter complementaryFilter;  // Complementary filter object for MPU6050 data

    // MPU6050 object (Adafruit_Sensor functionality)
    Adafruit_MPU6050 mpu;

    // Methods for data processing (mapping, etc.)
    float mapFlexToAngle(int sensorValue, int minSensorValue, int maxSensorValue, float minAngle, float maxAngle);
};

#endif // SENSOR_MANAGER_H
