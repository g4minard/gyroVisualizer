/*
  ==============================================================================
  | Receiver Sketch with GPS - Ground Station                                  |
  |----------------------------------------------------------------------------|
  | Receives sensor + GPS data wirelessly and outputs to Serial Monitor        |
  | in the format expected by the web dashboard.                               |
  | Upload this to the Arduino connected to your computer.                     |
  ==============================================================================
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- nRF24L01 Configuration ---
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001"; // Must match transmitter

// --- Data Structure ---
// MUST be identical to transmitter's struct
struct SensorData {
  float qW, qX, qY, qZ;
  float tempF;
  float accelG;
  float gpsSpeedMps;
  float altitudeFeet;
};

SensorData receivedData;

void setup() {
  // Start serial at high speed for web app
  Serial.begin(115200);
  while (!Serial);

  // Initialize radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  Serial.println("# Receiver ready - waiting for data...");
}

void loop() {
  if (radio.available()) {
    // Read the data structure
    radio.read(&receivedData, sizeof(SensorData));

    // Output in CSV format for web dashboard:
    // qw,qx,qy,qz,tempF,accelG,gpsSpeed,altitude
    Serial.print(receivedData.qW, 6); Serial.print(",");
    Serial.print(receivedData.qX, 6); Serial.print(",");
    Serial.print(receivedData.qY, 6); Serial.print(",");
    Serial.print(receivedData.qZ, 6); Serial.print(",");
    Serial.print(receivedData.tempF, 2); Serial.print(",");
    Serial.print(receivedData.accelG, 3); Serial.print(",");
    Serial.print(receivedData.gpsSpeedMps, 2); Serial.print(",");
    Serial.println(receivedData.altitudeFeet, 1);
  }
}