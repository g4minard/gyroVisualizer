/*
  ==============================================================================
  | Transmitter Sketch with GPS - Mobile Unit                                  |
  |----------------------------------------------------------------------------|
  | Reads MPU-9250 and NEO-6M GPS, performs sensor fusion, and sends           |
  | orientation quaternion, GPS velocity, and altitude via nRF24L01.           |
  ==============================================================================
*/

// Required Libraries:
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "MPU9250.h"

// --- Pin Configuration ---
const int NRF_CE_PIN = 9;
const int NRF_CSN_PIN = 10;
const int MPU_ADDRESS = 0x68;
const int GPS_RX_PIN = 4;  // GPS TX connects here
const int GPS_TX_PIN = 3;  // GPS RX connects here

// --- Global Objects ---
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
MPU9250 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// --- nRF24L01 Configuration ---
const byte radioAddress[6] = "00001";

// --- Data Structure ---
// This struct holds all sensor + GPS data
// MUST be identical on transmitter and receiver
struct SensorData {
  float qW, qX, qY, qZ;
  float tempF;
  float accelG;
  float gpsSpeedMps;     // GPS velocity in m/s
  float altitudeFeet;    // GPS altitude in feet
};
SensorData dataToSend;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600); // NEO-6M default baud rate
  
  // --- Initialize MPU-9250 ---
  Wire.begin();
  
  if (!mpu.setup(MPU_ADDRESS)) {  
    Serial.println("MPU connection failed. Check wiring!");
    while (1);
  }
  
  Serial.println("Calibrating MPU-9250. Keep sensor still...");
  mpu.calibrateAccelGyro();
  Serial.println("Accel/Gyro calibration complete.");
  
  Serial.println("Calibrate magnetometer - move in figure-8...");
  mpu.calibrateMag();
  Serial.println("Mag calibration complete.");

  // --- Initialize nRF24L01 Radio ---
  radio.begin();
  radio.openWritingPipe(radioAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  Serial.println("Transmitter setup complete. Waiting for GPS fix...");
}

void loop() {
  // Update GPS data (non-blocking)
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Update IMU data
  if (mpu.update()) {
    // Load quaternion data
    dataToSend.qW = mpu.getQuaternionW();
    dataToSend.qX = mpu.getQuaternionX();
    dataToSend.qY = mpu.getQuaternionY();
    dataToSend.qZ = mpu.getQuaternionZ();
    
    // Temperature (Celsius to Fahrenheit)
    dataToSend.tempF = mpu.getTemperature() * 9.0/5.0 + 32.0;
    
    // Linear acceleration magnitude in G's
    float ax = mpu.getLinearAccX();
    float ay = mpu.getLinearAccY();
    float az = mpu.getLinearAccZ();
    dataToSend.accelG = sqrt(ax*ax + ay*ay + az*az) / 9.807;

    // GPS data (use 0 if invalid)
    if (gps.speed.isValid()) {
      dataToSend.gpsSpeedMps = gps.speed.mps();
    } else {
      dataToSend.gpsSpeedMps = 0.0;
    }
    
    if (gps.altitude.isValid()) {
      dataToSend.altitudeFeet = gps.altitude.feet();
    } else {
      dataToSend.altitudeFeet = 0.0;
    }

    // Send the data packet
    bool result = radio.write(&dataToSend, sizeof(SensorData));
    
    // Optional debugging - uncomment to see transmitted data
    /*
    if (result) {
      Serial.print("Sent - Q: ");
      Serial.print(dataToSend.qW, 3); Serial.print(",");
      Serial.print(dataToSend.qX, 3); Serial.print(",");
      Serial.print(dataToSend.qY, 3); Serial.print(",");
      Serial.print(dataToSend.qZ, 3);
      Serial.print(" | Temp: "); Serial.print(dataToSend.tempF, 1);
      Serial.print("F | Accel: "); Serial.print(dataToSend.accelG, 2);
      Serial.print("G | Speed: "); Serial.print(dataToSend.gpsSpeedMps, 2);
      Serial.print("m/s | Alt: "); Serial.print(dataToSend.altitudeFeet, 1);
      Serial.print("ft | Sats: "); Serial.println(gps.satellites.value());
    } else {
      Serial.println("Transmission failed");
    }
    */
  }

  delay(10); // Small delay to prevent flooding
}