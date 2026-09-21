#include <Servo.h>
#include <CheapStepper.h>

// Use the Uno's hardware UART (pin 0 / RX) for the Seeed data.  Unlike
// SoftwareSerial, it does not disrupt the precisely timed Servo pulses.
const unsigned long DATA_BAUD = 57600;

// false: Serial Plotter output (only the servo target angle as a number).
// true:  human-readable diagnostics in the Serial Monitor.
const bool DEBUG_SERIAL = false;

const byte SERVO_PIN = 5;
const int SERVO_OPEN_ANGLE = 0;
const int SERVO_CLOSE_ANGLE = 20;

// Servo.write() is already called only on a state change.  The Servo library
// nevertheless keeps producing holding pulses.  Set this true to stop those
// pulses after the servo has reached its requested position.
const bool DETACH_SERVO_AFTER_MOVE = false;
const unsigned long SERVO_SETTLE_MS = 500;

Servo stateServo;
bool isClosed = true;
bool previousContraction = false;
int servoTargetAngle = SERVO_CLOSE_ANGLE;
bool servoAttached = false;
unsigned long servoDetachTime = 0;

// CheapStepper uses Arduino pins 8, 9, 10 and 11 by default (ULN2003
// IN1..IN4).  These pins do not conflict with the serial receiver or servo.
CheapStepper stepper;
const int STEPPER_RPM = 15;
const int ANGLE_STOP_LOW = 20;
const int ANGLE_STOP_HIGH = 70;

enum StepperDirection {
  STEPPER_STOPPED,
  STEPPER_CLOCKWISE,
  STEPPER_COUNTER_CLOCKWISE
};

StepperDirection stepperDirection = STEPPER_STOPPED;
unsigned long lastStepperStepTime = 0;
bool hasRollAngle = false;

bool contraction = false;
int rollAngle = 0;

void updateStepperDirection()
{
  // The stop region includes 20 and 70 degrees.
  if (rollAngle > ANGLE_STOP_HIGH)
    stepperDirection = STEPPER_COUNTER_CLOCKWISE;
  else if (rollAngle < ANGLE_STOP_LOW)
    stepperDirection = STEPPER_CLOCKWISE;
  else
    stepperDirection = STEPPER_STOPPED;
}

void runStepper()
{
  if (!hasRollAngle || stepperDirection == STEPPER_STOPPED)
    return;

  // CheapStepper::step() has no built-in rate control, so use the delay
  // calculated by setRpm() and take one non-blocking step at a time.
  unsigned long now = micros();
  if (now - lastStepperStepTime >= (unsigned long)stepper.getDelay())
  {
    stepper.step(stepperDirection == STEPPER_CLOCKWISE);
    lastStepperStepTime = now;
  }
}

void toggleServoState()
{
  isClosed = !isClosed;

  if (!servoAttached)
  {
    stateServo.attach(SERVO_PIN);
    servoAttached = true;
  }

  if (isClosed)
  {
    servoTargetAngle = SERVO_CLOSE_ANGLE;
  }
  else
  {
    servoTargetAngle = SERVO_OPEN_ANGLE;
  }

  stateServo.write(servoTargetAngle);

  if (DETACH_SERVO_AFTER_MOVE)
    servoDetachTime = millis() + SERVO_SETTLE_MS;

  if (DEBUG_SERIAL)
  {
    Serial.print("Servo: ");
    Serial.print(isClosed ? "closed, target: " : "open, target: ");
    Serial.println(servoTargetAngle);
  }
}

// Parse a packet produced by seeed_tx: "<0-or-1>,<angle>\n".
bool parsePacket(char *packet)
{
  char *comma = strchr(packet, ',');
  if (comma == NULL)
    return false;

  *comma = '\0';
  int receivedContraction = atoi(packet);
  int receivedAngle = atoi(comma + 1);

  if ((receivedContraction != 0 && receivedContraction != 1) ||
      receivedAngle < -180 || receivedAngle > 180)
    return false;

  contraction = (receivedContraction == 1);
  rollAngle = receivedAngle;
  hasRollAngle = true;
  updateStepperDirection();
  return true;
}

void setup()
{
  // Serial is both the hardware-UART input from the Seeed and the USB output
  // for Serial Monitor/Plotter.  Set the Monitor/Plotter to 57600 baud.
  Serial.begin(DATA_BAUD);
  stateServo.attach(SERVO_PIN);
  servoAttached = true;
  stateServo.write(servoTargetAngle);
  if (DETACH_SERVO_AFTER_MOVE)
    servoDetachTime = millis() + SERVO_SETTLE_MS;
  stepper.setRpm(STEPPER_RPM);

  if (DEBUG_SERIAL)
    Serial.println("Arduino receiver ready; initial state: closed.");
}

void loop()
{
  static char packet[16];
  static byte packetLength = 0;

  if (DETACH_SERVO_AFTER_MOVE && servoAttached &&
      (long)(millis() - servoDetachTime) >= 0)
  {
    stateServo.detach();
    servoAttached = false;
  }

  runStepper();

  while (Serial.available() > 0)
  {
    char received = Serial.read();

    if (received == '\r')
      continue;

    if (received == '\n')
    {
      packet[packetLength] = '\0';

      if (parsePacket(packet))
      {
        // A contraction is a rising edge, not every packet containing 1.
        // Thus 0,1,1,1,0 causes exactly one toggle.
        if (contraction && !previousContraction)
          toggleServoState();

        previousContraction = contraction;

        if (DEBUG_SERIAL)
        {
          Serial.print("Contraction: ");
          Serial.print(contraction ? 1 : 0);
          Serial.print(", Roll: ");
          Serial.print(rollAngle);
          Serial.print(", Servo target: ");
          Serial.println(servoTargetAngle);
        }
        else
        {
          // A bare number on every line is directly compatible with Serial
          // Plotter and represents the target last sent to the servo.
          Serial.println(servoTargetAngle);
        }
      }
      else
      {
        if (DEBUG_SERIAL)
          Serial.println("Invalid packet received.");
      }

      packetLength = 0;
    }
    else if (packetLength < sizeof(packet) - 1)
    {
      packet[packetLength++] = received;
    }
    else
    {
      // Discard an overlong/corrupted packet and wait for the next line.
      packetLength = 0;
    }
  }
}
