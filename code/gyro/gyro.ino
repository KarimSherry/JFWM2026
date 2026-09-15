#include "LSM6DS3.h"
#include "Wire.h"

// I2C address of the IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (myIMU.begin() != 0) {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  Serial.println("IMU initialized");
}

void loop() {

  float gyroX = myIMU.readFloatGyroX();
  float gyroY = myIMU.readFloatGyroY();
  float gyroZ = myIMU.readFloatGyroZ();

  Serial.print("X: ");
  Serial.print(gyroX, 2);

  Serial.print("  Y: ");
  Serial.print(gyroY, 2);

  Serial.print("  Z: ");
  Serial.println(gyroZ, 2);

  delay(500);
}
