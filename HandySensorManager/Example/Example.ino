#include <Wire.h>
#include <FlexLibrary.h>        // Include FlexLibrary for flex sensors
#include <Adafruit_MPU6050.h>   // Include Adafruit_Sensor for sensor functionality (e.g., MPU6050)
#include "SensorManager.h"      // Include the custom SensorManager class

// Define flex sensor pins
const int thumbPin = 32;
const int indexPin = 33;
const int middlePin = 34;
const int ringPin = 35;
const int pinkyPin = 36;

// Create an instance of the SensorManager
SensorManager sensorManager(thumbPin, indexPin, middlePin, ringPin, pinkyPin);

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  // Initialize sensors
  sensorManager.initializeSensors();

  // Calibrate sensors
  sensorManager.calibrateSensors();

}

void loop() {
  // Check if the calibration button is pressed and perform calibration if necessary
  //sensorManager.checkCalibrationButton();

  // Read flex sensor data
  sensorManager.readSensors();

  // Print the sensor data to the serial monitor
  sensorManager.printSensorData();

  // Read and process MPU6050 data
  sensorManager.readMPU6050Data();

  delay(500);  // Delay to allow for reading and processing
}
