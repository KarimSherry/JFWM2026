#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

// -------------------------
// Pins
// -------------------------
const int EMG_PIN = A0;

// EMG contraction threshold
const int EMG_THRESHOLD = 100;

// UART connected to the Arduino receiver.  On the Seeed board, Serial is
// kept for the USB Serial Monitor and Serial1 is the hardware UART.
const unsigned long DATA_BAUD = 57600;

// -------------------------
// IMU
// -------------------------
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// -------------------------
// Roll estimation
// -------------------------
float roll = 0.0;
float flippedRoll = 0.0;

unsigned long previousTime;

// Complementary filter
const float ALPHA = 0.98;

// Keep an angle in the range [-180, 180).  This is important when comparing
// two angles: 179 degrees and -179 degrees are only 2 degrees apart.
float wrapAngle(float angle)
{
  while (angle >= 180.0)
    angle -= 360.0;

  while (angle < -180.0)
    angle += 360.0;

  return angle;
}


void setup()
{
  Serial.begin(115200);
  Serial1.begin(DATA_BAUD);
  delay(1000);

  // Initialize IMU
  if (myIMU.begin() != 0)
  {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  Serial.println("IMU initialized");

  previousTime = micros();
}


void loop()
{
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

  // Determine contraction
  bool contraction = (emgValue >= EMG_THRESHOLD);


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
  // Calculate roll
  // -------------------------
  float accelRoll = atan2(ay, az) * 180.0 / PI;


  // -------------------------
  // Integrate gyro
  // -------------------------
  roll += gx * dt;


  // -------------------------
  // Complementary filter
  // -------------------------
  // Do not average the two absolute angles directly.  At the wrap point,
  // e.g. roll = 179 and accelRoll = -179, a direct average goes through 0.
  // Instead, apply the shortest angular correction from the accelerometer.
  float rollError = wrapAngle(accelRoll - roll);
  roll += (1.0 - ALPHA) * rollError;
  roll = wrapAngle(roll);

  // Keep the transmitted, flipped angle in the same [-180, 180) range.
  flippedRoll = wrapAngle(180.0 - roll);

  // -------------------------
  // Send one newline-terminated UART packet: contraction,angle
  // Example: "1,-42\n"
  // -------------------------
  Serial1.print(contraction ? 1 : 0);
//  Serial.println(emgValue);
  Serial1.print(',');
  Serial1.println(flippedRoll, 0);


  delay(10);  // ~100 Hz
}
