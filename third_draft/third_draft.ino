#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

// WiFi AP SSID
#define WIFI_SSID "wifiD1310-TIME2.4GHz"
//#define WIFI_SSID "Guest@APU"
//#define WIFI_SSID "Alsubai"
// WiFi password
#define WIFI_PASSWORD "11331010"
//#define WIFI_PASSWORD "Welcome2APU"
//#define WIFI_PASSWORD "tayv24566"

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "azpYCm2ktiFyuphO4StmR5RPqI33B9AndvLLKCXMMK1-eG1GaGhTUbw5FiHRGWapvLFPIALa2fseTbIdmW-R0Q=="
#define INFLUXDB_ORG "e11d11070e391b1e"
#define INFLUXDB_BUCKET "sensorTesting"

// Time zone info
#define TZ_INFO "UTC8"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

Point sensorData("sensor_readings");
Point wifiModule("wifi_status");

#if defined(ARDUINO_ESP32S2_DEV) || defined(ARDUINO_ESP32S3_DEV)
const int lowestPin = 1;
const int highestPin = 42;
#elif defined(ARDUINO_ESP32C3_DEV)
const int lowestPin = 1;
const int highestPin = 19;
#else
const int lowestPin = 2;
const int highestPin = 33;
#endif

//Necessary Libraries
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include "FlexLibrary.h"
#include "SensorManager.h" 

Servo myservo;
Adafruit_MPU6050 mpu;
static const int servoPin = 23;
const int voltagePin = 25;

Flex thumbFlex(36);
Flex indexFlex(35);
Flex middleFlex(34);
Flex ringFlex(33);
Flex pinkyFlex(32);

void configureAccelerometerRange() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
}

// Function to set and print gyro range
void configureGyroRange() {
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+-250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+-500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+-1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+-2000 deg/s");
      break;
  }
}

// Function to set and print filter bandwidth
void configureFilterBandwidth() {
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);  // Give time for the setup to stabilize

  // Calibration process
  for (int i = 0; i < 1000; i++) {
    ringFlex.Calibrate();
    thumbFlex.Calibrate();
    pinkyFlex.Calibrate();
    middleFlex.Calibrate();
    indexFlex.Calibrate();
  }
  delay(100);

  myservo.attach(servoPin);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  configureAccelerometerRange();
  configureGyroRange();
  configureFilterBandwidth();

  Serial.println("");
  delay(100);

}

void loop() {
  ringFlex.updateVal();
  thumbFlex.updateVal();
  pinkyFlex.updateVal();
  middleFlex.updateVal();
  indexFlex.updateVal();

  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Clear fields before adding new data
  sensorData.clearFields();
  wifiModule.clearFields();

  // Add data to points
  sensorData.addField("Thumb Flex Sensor", readThumbFlexSensor());
  sensorData.addField("Index Flex Sensor", readIndexFlexSensor());
  sensorData.addField("Middle Flex Sensor", readMiddleFlexSensor());
  sensorData.addField("Ring Flex Sensor", readRingFlexSensor());
  sensorData.addField("Pinky Flex Sensor", readPinkyFlexSensor());

  sensorData.addField("Gyro X", g.gyro.x);
  sensorData.addField("Gyro Y", g.gyro.y);
  sensorData.addField("Gyro Z", g.gyro.z);

  sensorData.addField("Acceleration X", a.acceleration.x);
  sensorData.addField("Acceleration Y", a.acceleration.y);
  sensorData.addField("Acceleration Z", a.acceleration.z);

  sensorData.addField("Battery", readBatteryLevel());
  wifiModule.addField("RSSI", WiFi.RSSI());

  client.writePoint(sensorData);

  // Print what we are writing
  //Serial.print("Sending");
  Serial.println("Resistance: ");
  Serial.print(readRingFlexSensor());
  Serial.print(", ");
  Serial.print(readPinkyFlexSensor());

  Serial.println("Gyro  ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.print(g.gyro.z);

  Serial.println("Accel ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y); 
  Serial.print(", ");
  Serial.print(a.acceleration.z); 


  //Serial.println(client.pointToLineProtocol(sensorData));

  // Check WiFi connection and reconnect if needed
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }

  // Write point
  if (!client.writePoint(wifiModule) || 
      !client.writePoint(sensorData))  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.println("Waiting....");
  delay(100);
}

// Functions for reading flex sensors
float readThumbFlexSensor() {
  return thumbFlex.getSensorValue();
}
float readIndexFlexSensor() {
  return indexFlex.getSensorValue();
}
float readMiddleFlexSensor() {
  return middleFlex.getSensorValue();
}
float readRingFlexSensor() {
  return ringFlex.getSensorValue();
}
float readPinkyFlexSensor() {
  return pinkyFlex.getSensorValue();
}

float readBatteryLevel() {
  int sensorValue = analogRead(voltagePin);  // Read the analog value (0-4095 for ESP32)
  float batteryLevel = sensorValue * (3.3 / 4095.0);  // ESP32 ADC reference voltage is 3.3V
  return batteryLevel;
}