// I2Cdev and MPU6050 library by Jeff Rowberg
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
// --- NEW: Variables for acceleration data ---
VectorInt16 aa;         // [x, y, z]            accelerometer measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel


volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();
    pinMode(2, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // --- Get Yaw, Pitch, Roll ---
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // --- NEW: Get Temperature ---
        // The embedded temp sensor is not highly accurate but gives a good estimate.
        int16_t tempRaw = mpu.getTemperature();
        float tempC = (tempRaw / 340.0) + 36.53;
        float tempF = (tempC * 9.0/5.0) + 32.0;

        // --- NEW: Get Real Acceleration (without gravity) ---
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        // Calculate magnitude in G's
        // Sensitivity is 16384 LSB/g for the default +/- 2g range
        float accel_x = aaReal.x / 16384.0;
        float accel_y = aaReal.y / 16384.0;
        float accel_z = aaReal.z / 16384.0;
        float accelMagnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

        // --- UPDATED: Send all 5 data points ---
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print(",");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print(",");
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print(",");
        Serial.print(tempF);
        Serial.print(",");
        Serial.println(accelMagnitude);
    }
}

