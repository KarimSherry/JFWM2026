#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

// -------------------------
// Pins
// -------------------------
const int EMG_PIN = A0;

// -------------------------
// IMU
// -------------------------
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// -------------------------
// Roll estimation
// -------------------------
float roll = 0.0;

unsigned long previousTime;

// Complementary filter
const float ALPHA = 0.98;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  Serial.println("IMU initialized");

  previousTime = micros();
}

void loop() {

  // -------------------------
  // Time step
  // -------------------------
  unsigned long currentTime = micros();

  float dt = (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  // -------------------------
  // Read EMG
  // -------------------------
  int emgValue = analogRead(EMG_PIN);

  // -------------------------
  // Read accelerometer
  // -------------------------
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();

  // -------------------------
  // Read gyroscope
  // -------------------------
  float gx = myIMU.readFloatGyroX();

  // -------------------------
  // Calculate roll from accelerometer
  // -------------------------
  float accelRoll = atan2(ay, az) * 180.0 / PI;

  // -------------------------
  // Integrate gyro
  // -------------------------
  roll += gx * dt;

  // -------------------------
  // Complementary filter
  // -------------------------
  roll = ALPHA * roll + (1.0 - ALPHA) * accelRoll;

  // -------------------------
  // Output
  // -------------------------
  Serial.print("EMG:");
  Serial.print(emgValue);

  Serial.print(", Roll:");
  Serial.println(roll, 2);

  delay(10);  // ~100 Hz
}
