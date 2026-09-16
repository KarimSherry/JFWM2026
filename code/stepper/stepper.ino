#include <CheapStepper.h>

CheapStepper stepper;

// Motor state
enum MotorState {
  STOPPED,
  CLOCKWISE,
  COUNTER_CLOCKWISE
};

MotorState motorState = STOPPED;

void setup()
{
  Serial.begin(9600);

  // Maximum speed you found to be reliable
  stepper.setRpm(20);

  Serial.println("Stepper ready.");
  Serial.println("c = clockwise");
  Serial.println("a = counter-clockwise");
  Serial.println("s = stop");
}

void loop()
{
  // -------------------------
  // Check keyboard command
  // -------------------------
  if (Serial.available() > 0)
  {
    char command = Serial.read();

    if (command == 'c' || command == 'C')
    {
      motorState = CLOCKWISE;
      Serial.println("Clockwise");
    }

    else if (command == 'a' || command == 'A')
    {
      motorState = COUNTER_CLOCKWISE;
      Serial.println("Counter-clockwise");
    }

    else if (command == 's' || command == 'S')
    {
      motorState = STOPPED;
      Serial.println("Stopped");
    }
  }

  // -------------------------
  // Move motor
  // -------------------------
  if (motorState == CLOCKWISE)
  {
    stepper.step(true);
  }

  else if (motorState == COUNTER_CLOCKWISE)
  {
    stepper.step(false);
  }

  // If STOPPED, do nothing
}
