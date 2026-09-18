#include <SoftwareSerial.h>
#include <Servo.h>
#include <CheapStepper.h>

// Keep Serial for the USB Serial Monitor.  The Seeed transmitter connects to
// this Arduino's pin 2; pin 3 is unused here but required by SoftwareSerial.
const byte DATA_RX_PIN = 2;
const byte DATA_TX_PIN = 3;
const unsigned long DATA_BAUD = 57600;

SoftwareSerial dataSerial(DATA_RX_PIN, DATA_TX_PIN);

const byte SERVO_PIN = 6;
const int SERVO_OPEN_ANGLE = 0;
const int SERVO_CLOSE_ANGLE = 20;

Servo stateServo;
bool isClosed = true;
bool previousContraction = false;

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

  if (isClosed)
  {
    stateServo.write(SERVO_CLOSE_ANGLE);
    Serial.println("Servo: closed");
  }
  else
  {
    stateServo.write(SERVO_OPEN_ANGLE);
    Serial.println("Servo: open");
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
  Serial.begin(115200);
  dataSerial.begin(DATA_BAUD);
  stateServo.attach(SERVO_PIN);
  stateServo.write(SERVO_CLOSE_ANGLE);
  stepper.setRpm(STEPPER_RPM);

  Serial.println("Arduino receiver ready; initial state: closed.");
}

void loop()
{
  static char packet[16];
  static byte packetLength = 0;

  runStepper();

  while (dataSerial.available() > 0)
  {
    char received = dataSerial.read();

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
      }
      else
      {
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
