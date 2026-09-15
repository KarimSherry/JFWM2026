#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

// XIAO nRF52840 Sense IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Orientation angles
float roll = 0.0;
float pitch = 0.0;

// Timing
unsigned long previousTime;

// Complementary filter coefficient
// Higher = trust gyro more
// Lower  = trust accelerometer more
const float ALPHA = 0.98;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (myIMU.begin() != 0) {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  Serial.println("IMU initialized");

  // Initialize timing
  previousTime = micros();
}

void loop() {

  // --------------------------------------------------
  // 1. Calculate elapsed time
  // --------------------------------------------------

  unsigned long currentTime = micros();

  float dt = (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  // --------------------------------------------------
  // 2. Read accelerometer
  // --------------------------------------------------

  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();

  // --------------------------------------------------
  // 3. Read gyroscope
  // --------------------------------------------------

  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  // --------------------------------------------------
  // 4. Calculate orientation from accelerometer
  // --------------------------------------------------

  float accelRoll =
      atan2(ay, az) * 180.0 / PI;

  float accelPitch =
      atan2(
          -ax,
          sqrt(ay * ay + az * az)
      ) * 180.0 / PI;

  // --------------------------------------------------
  // 5. Integrate gyro
  // --------------------------------------------------

  roll += gx * dt;
  pitch += gy * dt;

  // --------------------------------------------------
  // 6. Complementary filter
  // --------------------------------------------------

  roll =
      ALPHA * roll +
      (1.0 - ALPHA) * accelRoll;

  pitch =
      ALPHA * pitch +
      (1.0 - ALPHA) * accelPitch;

  // --------------------------------------------------
  // 7. Print results
  // --------------------------------------------------

  Serial.print("Roll: ");
  Serial.print(roll, 2);

  Serial.print("  Pitch: ");
  Serial.print(pitch, 2);

  Serial.print("  dt: ");
  Serial.println(dt, 4);

  delay(10);  // approximately 100 Hz
}
