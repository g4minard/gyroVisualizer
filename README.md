# MPU-6050 Real-Time 3D Web Visualizer

A real-time 3D web visualizer for the MPU-6050 accelerometer and gyroscope, communicating directly from an Arduino Nano to a web browser using the Web Serial API.

## Features

-   **Real-Time Visualization:** 3D model rotates to exactly match the sensor's orientation.
-   **Live Data Readouts:** Displays Yaw, Pitch, Roll, Temperature (°F), and user-calibrated Acceleration (G).
-   **Browser-Based:** No software needed other than a modern web browser (like Chrome or Edge) that supports the Web Serial API.
-   **Zero Calibration:** A "Zero" button allows the user to set the current orientation and acceleration as the baseline.

## Hardware Required

-   Arduino Nano (or similar)
-   MPU-6050 Gyroscope/Accelerometer Module
-   Jumper Wires

## Software & Libraries

-   [Arduino IDE](https://www.arduino.cc/en/software)
-   MPU6050 Library by Jeff Rowberg ([I2Cdevlib](https://github.com/jrowberg/i2cdevlib))

## How To Use

1.  **Upload the Sketch:** Open the `arduino_sketch/arduino_sketch.ino` file in the Arduino IDE. Make sure you have the required libraries installed, then upload it to your Arduino.
2.  **Open the Visualizer:** Open the `index.html` file in Google Chrome or Microsoft Edge.
3.  **Connect:** Click the "Connect" button and select the COM port that your Arduino is connected to from the pop-up list.
4.  **Visualize:** The 3D model and data readouts should now be active. Move the sensor to see it update in real time!
