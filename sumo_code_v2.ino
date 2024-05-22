#include <Motoron.h>

MotoronI2C mc;

const auto vinType = MotoronVinSenseType::MotoronHp;
const uint16_t referenceMv = 5000;
const uint16_t minVinVoltageMv = 6500;

const int CENTER_PIN = 1;
const int LEFT_X_PIN = 2;
const int RIGHT_X_PIN = 3;
const int PARALLEL_LEFT_PIN = 4;
const int PARALLEL_RIGHT_PIN = 5;

const int LINE_FRONT_LEFT = 6;
const int LINE_FRONT_RIGHT = 7;
const int LINE_BACK_LEFT = 8;
const int LINE_BACK_RIGHT = 9;

  bool centerDetected;
  bool leftXDetected;
  bool rightXDetected;
  bool parallelLeftDetected;
  bool parallelRightDetected;
  bool frontLeft;
  bool frontRight;
  bool backLeft;
  bool backRight;

int check_state = 0;
const int KILL = 11;

const uint16_t errorMask = 
  (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR) |
  (1 << MOTORON_STATUS_FLAG_CRC_ERROR) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_RESET) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER) |
  (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE);

void checkCommunicationError(uint8_t errorCode)
{
  if (errorCode)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("Communication error: "));
      Serial.println(errorCode);
      delay(1000);
    }
  }
}

void checkControllerError(uint16_t status)
{
  if (status & errorMask)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("Controller error: 0x"));
      Serial.println(status, HEX);
      delay(1000);
    }
  }
}

void checkVinVoltage(uint16_t voltageMv)
{
  if (voltageMv < minVinVoltageMv)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("VIN voltage too low: "));
      Serial.println(voltageMv);
      delay(1000);
    }
  }
}

void checkForProblems() {
  uint16_t status = mc.getStatusFlags();
  checkCommunicationError(mc.getLastError());
  checkControllerError(status);

  uint32_t voltageMv = mc.getVinVoltageMv(referenceMv, vinType);
  checkCommunicationError(mc.getLastError());
  checkVinVoltage(voltageMv);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mc.reinitialize();
  mc.clearResetFlag();

  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_BRAKE);
  mc.setErrorMask(errorMask);

  mc.setCommandTimeoutMilliseconds(100);

  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 300);

  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);

  while(mc.getMotorDrivingFlag())
  mc.clearMotorFaultUnconditional();

  pinMode(CENTER_PIN, INPUT);
  pinMode(LEFT_X_PIN, INPUT);
  pinMode(RIGHT_X_PIN, INPUT);
  pinMode(PARALLEL_LEFT_PIN, INPUT);
  pinMode(PARALLEL_RIGHT_PIN, INPUT);

  pinMode(LINE_FRONT_LEFT, INPUT);
  pinMode(LINE_FRONT_RIGHT, INPUT);
  pinMode(LINE_BACK_LEFT, INPUT);
  pinMode(LINE_BACK_RIGHT, INPUT);

  pinMode(KILL, INPUT);
}


int PercentToSpeed(int speed, String direction) {
  if (speed > 100) speed = 100; // Clamp speed to max 100
  if (speed < 0) speed = 0; // Clamp speed to min 0

  int mappedSpeed = map(speed, 0, 100, 0, 800);

  if (direction == "back") mappedSpeed *= -1;

  return mappedSpeed;
}

void MoveForwardFULL() {
  int percentage = PercentToSpeed(100, "front");
  mc.setSpeed(1, percentage);
  mc.setSpeed(2, percentage);
}

void MoveForward() {
  int percentage = PercentToSpeed(75, "front");
  mc.setSpeed(1, percentage);
  mc.setSpeed(2, percentage);
}

void MoveBackward() {
  int percentage = PercentToSpeed(100, "back");
  mc.setSpeed(1, percentage);
  mc.setSpeed(2, percentage);
}

void RotateLeft() {
  int percentage = PercentToSpeed(40, "front");
  mc.setSpeed(1, percentage);
  mc.setSpeed(2, -percentage);
}

void RotateRight() {
  int percentage = PercentToSpeed(40, "back");
  mc.setSpeed(1, -percentage);
  mc.setSpeed(2, percentage);
}

void NoDetect() {
  int percentage = PercentToSpeed(40, "back");
  mc.setSpeed(1, percentage);
  mc.setSpeed(2, percentage);
}

void BeginSearch() {
  centerDetected = digitalRead(CENTER_PIN);
  leftXDetected = digitalRead(LEFT_X_PIN);
  rightXDetected = digitalRead(RIGHT_X_PIN);
  parallelLeftDetected = digitalRead(PARALLEL_LEFT_PIN);
  parallelRightDetected = digitalRead(PARALLEL_RIGHT_PIN);
  frontLeft = digitalRead(LINE_FRONT_LEFT);
  frontRight = digitalRead(LINE_FRONT_RIGHT);
  backLeft = digitalRead(LINE_BACK_LEFT);
  backRight = digitalRead(LINE_BACK_RIGHT);

  if (centerDetected && leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForwardFULL();
  }

  if (centerDetected && leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && (backLeft || backRight)) {
    MoveForwardFULL();
  }

  if (centerDetected && leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForwardFULL();
  }

  if (centerDetected && !leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForwardFULL();
  }

  if (centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForward();
  }

  if (!centerDetected && leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForward();
  }

  if (!centerDetected && !leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    MoveForward();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    RotateLeft();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    RotateRight();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && !frontLeft && !frontRight && !backLeft && !backRight) {
    NoDetect();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && (frontLeft || frontRight)) {
    MoveBackward();
    RotateLeft();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected && (backLeft || backRight)) {
    MoveBackward();
    RotateLeft();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && parallelLeftDetected && !parallelRightDetected && (frontRight || backRight) && !frontLeft && !backLeft) {
    RotateLeft();
    MoveForward();
  }

  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && parallelRightDetected && (frontLeft || backLeft) && !frontRight && !backRight) {
    RotateRight();
    MoveForward();
  }
}


 void loop() {
  checkForProblems();
  BeginSearch();

}
