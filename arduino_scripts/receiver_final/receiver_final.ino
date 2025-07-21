/*
  Final Receiver Sketch
  Receives sensor data wirelessly and prints it to the Serial Monitor
  in the format expected by the web application.
  Upload this to the Arduino connected to your computer.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 object
RF24 radio(9, 10); // CE, CSN

// nRF24L01 settings
const byte address[6] = "00001"; // Must be the same on both receiver and transmitter

// A data structure to hold all our sensor values.
// This MUST be identical to the struct on the transmitter.
struct SensorData {
  float qW, qX, qY, qZ;
  float tempF;
  float accelG;
};

SensorData receivedData;

void setup() {
  // Start serial at the high speed required by the web app
  Serial.begin(115200);
  while (!Serial);

  // Initialize Radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    // Read the data structure
    radio.read(&receivedData, sizeof(SensorData));

    // Print the data in the comma-separated format the web app expects
    Serial.print(receivedData.qW, 6); Serial.print(",");
    Serial.print(receivedData.qX, 6); Serial.print(",");
    Serial.print(receivedData.qY, 6); Serial.print(",");
    Serial.print(receivedData.qZ, 6); Serial.print(",");
    Serial.print(receivedData.tempF); Serial.print(",");
    Serial.println(receivedData.accelG);
  }
}
