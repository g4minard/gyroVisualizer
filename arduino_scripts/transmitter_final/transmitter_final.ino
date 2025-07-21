/*
  Final 9-DOF Transmitter Sketch
  Reads MPU-9250 sensor data (Accel, Gyro, Mag), performs on-board
  sensor fusion, and transmits the drift-corrected data wirelessly.
  
  This sketch uses the "MPU9250" library by Bolder Flight Systems.
*/

// MPU9250 Library by Bolder Flight Systems
#include "MPU9250.h"

// nRF24L01 Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// MPU9250 object
MPU9250 mpu;

// nRF24L01 object
RF24 radio(9, 10); // CE, CSN

// nRF24L01 settings
const byte address[6] = "00001"; // Must be the same on both receiver and transmitter

// A data structure to hold all our sensor values.
// This is identical to the struct on the receiver sketch.
struct SensorData {
  float qW, qX, qY, qZ;
  float tempF;
  float accelG;
};

SensorData dataToSend;

void setup() {
  // Start serial for debugging this unit, if needed
  Serial.begin(115200);
  while(!Serial);

  // Start the MPU-9250
  if (!mpu.setup(0x68)) {  // Use address 0x68, the default for MPU-9250
    while (1) {
      Serial.println("MPU connection failed. Check wiring.");
      delay(5000);
    }
  }

  // Calibrate gyroscope and accelerometer.
  // It's best to do this with the sensor still and level.
  mpu.calibrateAccelGyro();

  // Calibrate magnetometer.
  // For best results, you should perform a calibration dance:
  // slowly rotate the sensor in figure-8 patterns in the air
  // for about 30 seconds after this message appears.
  // For initial testing, you can skip this dance.
  Serial.println("Calibrating magnetometer. Please wait...");
  mpu.calibrateMag();

  // Initialize Radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Serial.println("Transmitter setup complete.");
}

void loop() {
  // Check if data is available
  if (mpu.update()) {
    // Load the quaternion data directly from the fusion algorithm
    dataToSend.qW = mpu.getQuaternionW();
    dataToSend.qX = mpu.getQuaternionX();
    dataToSend.qY = mpu.getQuaternionY();
    dataToSend.qZ = mpu.getQuaternionZ();

    // Get Temperature
    dataToSend.tempF = mpu.getTemperature_F();

    // Get Linear Acceleration (gravity-free)
    dataToSend.accelG = sqrt(pow(mpu.getLinearAccX_mss(), 2) +
                             pow(mpu.getLinearAccY_mss(), 2) +
                             pow(mpu.getLinearAccZ_mss(), 2)) / 9.807;

    // Send the data structure wirelessly
    radio.write(&dataToSend, sizeof(SensorData));
  }
}
