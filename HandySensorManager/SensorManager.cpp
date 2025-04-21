#include "SensorManager.h"

#define CALIBRATION_BUTTON_PIN 7

// Constructor definition: Initializes flex sensor pins and complementary filter
SensorManager::SensorManager(int thumbPin, int indexPin, int middlePin, int ringPin, int pinkyPin)
    : thumbFlex(thumbPin), indexFlex(indexPin), middleFlex(middlePin), ringFlex(ringPin), pinkyFlex(pinkyPin), complementaryFilter() {}

// Initializes sensors (FlexLibrary and others should be initialized in .ino)
void SensorManager::initializeSensors() {
    Serial.begin(115200);

    // Initialize the flex sensors (Note: done in .ino)
    // Initialize MPU6050 (Adafruit_Sensor functionality)
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050");
        while (1);  // Stop execution if MPU6050 initialization fails
    }
    Serial.println("MPU6050 initialized");

    // Initialize Complementary Filter
    complementaryFilter.begin();

}

// Calibrate the flex sensors
void SensorManager::calibrateSensors() {
    Serial.println("Starting Calibration...");

}

// Read flex sensor values (updated in .ino)
void SensorManager::readSensors() {
    thumbFlex.updateVal();
    indexFlex.updateVal();
    middleFlex.updateVal();
    ringFlex.updateVal();
    pinkyFlex.updateVal();

}

// Print the sensor data (flex sensor angles and MPU6050 data)
void SensorManager::printSensorData() {
    // Convert flex sensor values to angles minInput maxInput
    float thumbAngle = mapFlexToAngle(thumbFlex.getSensorValue(), 800, 351, 81, 0);
    Serial.print("Thumb Flex: ");
    Serial.println(thumbFlex.getSensorValue());
    Serial.println(thumbAngle);

    float indexAngle = mapFlexToAngle(indexFlex.getSensorValue(), 1804, 944, 104, 0);
    Serial.print("Index Flex: ");
    Serial.println(indexFlex.getSensorValue());
    Serial.println(indexAngle);

    float pinkyAngle = mapFlexToAngle(pinkyFlex.getSensorValue(), 3089, 2089, 104, 0);
    Serial.print("Pinky Flex: ");
    Serial.println(pinkyFlex.getSensorValue());
    Serial.println(pinkyAngle);

    float middleAngle = mapFlexToAngle(middleFlex.getSensorValue(), 1806, 666, 107, 0);
    Serial.print("Middle Flex: ");
    Serial.println(middleFlex.getSensorValue());
    Serial.println(middleAngle);

    float ringAngle = mapFlexToAngle(ringFlex.getSensorValue(), 1767, 720, 107, 0);
    Serial.print("Ring Flex: ");
    Serial.println(ringFlex.getSensorValue());
    Serial.println(ringAngle);

    // Print filtered data from Complementary Filter
    Serial.print("Roll: ");
    Serial.print(complementaryFilter.getRoll());
    Serial.print("°\tPitch: ");
    Serial.print(complementaryFilter.getPitch());
    Serial.print("°\tYaw: ");
    Serial.print(complementaryFilter.getYaw());
    Serial.println("°");

}

// Check if the calibration button is pressed (debounced)
void SensorManager::checkCalibrationButton() {
    static unsigned long lastPress = 0;
    unsigned long currentTime = millis();
    if (digitalRead(CALIBRATION_BUTTON_PIN) == LOW) {
        Serial.println("Calibration button pressed. Re-calibrating...");
        calibrateSensors();
    }

}


// Read MPU6050 data (processed by Complementary Filter)
void SensorManager::readMPU6050Data() {
    complementaryFilter.update();

}

// Helper method to map flex sensor readings to angles
float SensorManager::mapFlexToAngle(int sensorValue, int minSensorValue, int maxSensorValue, float minAngle, float maxAngle) {
    return ((float)(sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxAngle - minAngle) + minAngle;

}
