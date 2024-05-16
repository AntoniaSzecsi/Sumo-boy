#include <Motoron.h>

MotoronI2C mc;

const auto vinType = MotoronVinSenseType::MotoronHp;
const uint16_t referenceMv = 5000;
const uint16_t minVinVoltageMv = 6500;
const int sensorPin1 = A0;
const int sensorPin2 = A1;
int sensorValue1;
int sensorValue2;


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

  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);

}

void loop() {
  checkForProblems();
  
  BeginSearch();
   
  /* sensorValue1 = digitalRead(sensorPin1);
   sensorValue2 = digitalRead(sensorPin2);

  if (sensorValue1 == 1) {
    mc.setSpeed(1, 200);}
 
 if (sensorValue2 == 1) {
    mc.setSpeed(2, 200);
  }
 
  if (sensorValue1 == 0) {
    mc.setSpeed(1, 0);}
 
 if (sensorValue2 == 0) {
    mc.setSpeed(2, 0);
  }

  if(sensorValue1==1 && sensorValue2==1) {
  mc.setSpeed(1, 200);
  mc.setSpeed(2, 200);
  }
  
  if(sensorValue1==0 && sensorValue2==0) {
    mc.setSpeed(1, 0);
    mc.setSpeed(2, 0);
  }  
  */
}

void MoveForwardFULL(){
  mc.setSpeed(1 , 800);
  mc.setSpeed(2 , 800);
}

void MoveForward(){
  mc.setSpeed(1 , 600);
  mc.setSpeed(2 , 600);
  //se testeaza si asta
}

void MoveBackward(){
  mc.setSpeed(1 , -800);
  mc.setSpeed(2 , -800);
}

void RotateLeft(){
  mc.setSpeed(1 , 400);
  mc.setSpeed(2 ,-400);
  //se testeaza cat de mult se merge
}

void RotateRight(){
  mc.setSpeed(1 , -400);
  mc.setSpeed(2 , 400);
  //se testeaza cat de mult se merge
}

void NoDetect(){
  mc.setSpeed(1 , -400);
  mc.setSpeed(2 , 400);
  //se testeaza cat de mult se merge
}


void BeginSearch() {
  bool centerDetected = digitalRead(CENTER_PIN);
  bool leftXDetected = digitalRead(LEFT_X_PIN);
  bool rightXDetected = digitalRead(RIGHT_X_PIN);
  bool parallelLeftDetected = digitalRead(PARALLEL_LEFT_PIN);
  bool parallelRightDetected = digitalRead(PARALLEL_RIGHT_PIN);
  

  if (centerDetected && leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected) {
    // Center and right_x and left_x sensors detect
    MoveForrwardFULL();
  }
  if (centerDetected && leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected) {
    // Center and left_x sensors detect
    MoveForrwardFULL();
  }
  if (centerDetected && !leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected) {
    // Center and right_x sensors detect
    MoveForrwardFULL();
  }
  if (centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected) {
    // Only center sensor detects
    MoveForward();
  }
  if (!centerDetected && leftXDetected && !rightXDetected) && !parallelLeftDetected && !parallelRightDetected {
    // Only left_x sensor detects
    MoveForward();
  }
  if (!centerDetected && !leftXDetected && rightXDetected && !parallelLeftDetected && !parallelRightDetected) {
    // Only right_x sensor detects
    MoveForward();
  }
  if (!centerDetected && !leftXDetected && !rightXDetected && parallelLeftDetected && !parallelRightDetected) {
    // Only parallel left sensor detects
    RotateLeft();
  }
  if (!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && parallelRightDetected) {
    // Only parallel right sensor detects
    RotateRight();
  }
  if(!centerDetected && !leftXDetected && !rightXDetected && !parallelLeftDetected && !parallelRightDetected)
  //doesnt detect with no sensor
    NoDetect();
}



