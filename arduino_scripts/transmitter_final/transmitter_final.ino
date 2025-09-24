/*
  ==============================================================================
  | Final Transmitter Sketch - Corrected for your MPU9250 library             |
  |----------------------------------------------------------------------------|
  | Reads MPU-9250, performs sensor fusion, and sends the orientation          |
  | quaternion and other data wirelessly via an nRF24L01 module.               |
  | Upload this to the Arduino on your mobile unit.                            |
  ==============================================================================
*/

// Required Libraries:
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> // Explicitly include the Wire library for I2C
#include "MPU9250.h" // MPU9250 library

// --- Pin Configuration ---
const int NRF_CE_PIN = 9;
const int NRF_CSN_PIN = 10;
const int MPU_ADDRESS = 0x68;

// --- Global Objects ---
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
MPU9250 mpu; // Create the MPU object

// --- nRF24L01 Configuration ---
const byte radioAddress[6] = "00001"; // Must match the receiver's address

// --- Data Structure ---
// This struct holds all the sensor data we want to send.
// It MUST be identical on both the transmitter and receiver sketches.
struct SensorData {
  float qW, qX, qY, qZ;
  float tempF;
  float accelG;
};
SensorData dataToSend;


void setup() {
  Serial.begin(115200); // For debugging the transmitter itself
  
  // --- Initialize MPU-9250 ---
  Wire.begin(); // Start the I2C bus
  
  // Use the setup function
  if (!mpu.setup(MPU_ADDRESS)) {  
    Serial.println("MPU connection failed. Check wiring or I2C address.");
    while (1); // Halt execution
  }
  
  // The library you have doesn't use setAccelRange, setGyroRange, or setFilterBandwidth
  // Those settings are typically configured internally by the library
  // If you need to adjust settings, check your library's documentation
  
  Serial.println("Calibrating MPU-9250. Keep the sensor still...");
  mpu.calibrateAccelGyro();
  Serial.println("Accel/Gyro calibration complete.");
  
  Serial.println("Now calibrate magnetometer - move sensor in figure-8 pattern...");
  mpu.calibrateMag();
  Serial.println("Mag calibration complete.");

  // --- Initialize nRF24L01 Radio ---
  radio.begin();
  radio.openWritingPipe(radioAddress);
  radio.setPALevel(RF24_PA_MIN); // Use minimum power for short-range testing
  radio.stopListening();
  
  Serial.println("Transmitter setup complete. Sending data...");
}

void loop() {
  // Check if new data is available from the sensor
  if (mpu.update()) {
    // Load the quaternion data into our struct
    dataToSend.qW = mpu.getQuaternionW();
    dataToSend.qX = mpu.getQuaternionX();
    dataToSend.qY = mpu.getQuaternionY();
    dataToSend.qZ = mpu.getQuaternionZ();
    
    // Get temperature and convert Celsius to Fahrenheit
    dataToSend.tempF = mpu.getTemperature() * 9.0/5.0 + 32.0;
    
    // Calculate total linear acceleration magnitude (gravity compensated)
    // Get linear acceleration (gravity-free) in m/s^2
    float ax = mpu.getLinearAccX();
    float ay = mpu.getLinearAccY();
    float az = mpu.getLinearAccZ();
    
    // Calculate magnitude and convert to G's (divide by 9.807 m/s^2)
    dataToSend.accelG = sqrt(ax*ax + ay*ay + az*az) / 9.807;

    // Send the data packet
    bool result = radio.write(&dataToSend, sizeof(SensorData));
    
    // Optional: Uncomment for debugging
    /*
    if (result) {
      Serial.print("Sent - Q: ");
      Serial.print(dataToSend.qW, 3); Serial.print(", ");
      Serial.print(dataToSend.qX, 3); Serial.print(", ");
      Serial.print(dataToSend.qY, 3); Serial.print(", ");
      Serial.print(dataToSend.qZ, 3);
      Serial.print(" | Temp: "); Serial.print(dataToSend.tempF, 1);
      Serial.print("F | Accel: "); Serial.print(dataToSend.accelG, 2);
      Serial.println("G");
    } else {
      Serial.println("Transmission failed");
    }
    */
  }

  delay(10); // Small delay to prevent flooding the receiver
}