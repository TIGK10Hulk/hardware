// INCLUDES
#include <Arduino.h>
#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

// DECLARATIONS
  MeBuzzer buzzer;
  MeUltrasonicSensor ultraSensor(PORT_10);
  MeLineFollower line(PORT_9);
  MeEncoderOnBoard Encoder_1(SLOT1);
  MeEncoderOnBoard Encoder_2(SLOT2);
  MeEncoderMotor encoders[2];
  MeBluetooth bluetooth(PORT_3);
  MeGyro gyro (0,0x69);

// FUNCTIONS
  void isr_process_encoder1(void);
  void isr_process_encoder2(void);
  void Forward(void);
  void Backward(void);
  void TurnLeft(void);
  void TurnRight(void);
  void TurnLeft1(void);
  void TurnRight1(void);
  void Stop(void);
  double get_power(void);
  void lineFollower(void);
  void getBluetoothData(void);
  bool getDriveState(void); //get drive state through inputArr[0]; Manual = true, auto = false
  void setManualDriveCommands(int input);
  void checkObstacle(void);
  void sendBluetoothData(double protocol, double data1, double data2);
  void sendCoord(int, unsigned int);
  void angleCalculation(void);
  void calculateCoords(double distance);
  void randomLeftOrRight(int16_t delayInput);
  
// VARIABLES
int16_t moveSpeed = 150; // 100 = 0.214m/s, 200 = 0.43m/s
int16_t auriga_power = 0;
int16_t rampInterval = 750;
byte inputArr[2] = {0}; //BT input, 0 = manual/auto, 1 = drive commands, 2 = speed(0-255)
byte outputArr[3] = {0}; //BT output, 0 = manual/auto, 1 = speed(0-255), 2 = coord state, 3 X-coord, 4 = Y-coord, 5 = power
int16_t test = 0;

int16_t randomNumber = 0;

double totalAngle = 0, newAngle = 0;
double X = 0, Y = 0;
double distance = 0;

int oldInput = 9;

unsigned long deltaTime = 0, startTime = 0, stopTime = 0;
bool isMovingForward = false;
bool isMovingBackward = false;

// DEFINES
#define POWER_PORT  A4

void setup()  // put your setup code here, to run once:
{
  delay(5);
  Serial.begin(115200);
  gyro.begin();
  //bluetooth.begin(115200);  //The factory default baud rate is 115200
  Serial.println("Setup complete!");
  delay(5);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);  

  delay(5);
  pinMode(13,OUTPUT);
  encoders[0] = MeEncoderMotor(SLOT_1);
  encoders[1] = MeEncoderMotor(SLOT_2);
  encoders[0].begin();
  encoders[1].begin();

  encoders[0].runSpeed(0);
  encoders[1].runSpeed(0);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);
}

/**
 * \par Description
 *    This function use to process the interrupt of encoder1 drvicer on board,
 *    used to calculate the number of pulses.
 * \par Output
 *    The number of pulses on encoder1 driver
 */
void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }
  else{
    Encoder_1.pulsePosPlus();
  }
}

/**
 * \par Description
 *    This function use to process the interrupt of encoder2 drvicer on board,
 *    used to calculate the number of pulses.
 * \par Output
 *    The number of pulses on encoder2 driver
 */
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }
  else{
    Encoder_2.pulsePosPlus();
  }
}

/**
 * \par Description
 *    This function use to control the car kit go forward.
 */
void Forward(void)
{
  //Set isMoving to true
  isMovingForward = true;
  //Start measuring time interval
  startTime = millis();
  //Calculate angle
  angleCalculation();
  
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward.
 */
void Backward(void)
{
  //Set isMoving to true
  isMovingBackward = true;
  //Start measuring time interval
  startTime = millis();
  //Calculate angle
  angleCalculation();
  
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward and turn left(fast).
 */
void SpinLeft(void)
{
  //reset gyro
  gyro.resetZ();
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward and turn right(fast).
 */
void SpinRight(void)
{
  //reset gyro
  gyro.resetZ();
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Description
 *    This function use to stop the car kit.
 */
void Stop(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  //If prevStartTime = startTime then return
  if(isMovingForward != true || isMovingBackward != true)
  {
    return;   
  }
  else
  {
    //Stop measuring time interval
    stopTime = millis();
    //Calculate time interval
    deltaTime = stopTime - startTime;
    //Calculate distance with time interval, convert arbitrary moveSpeed to cm/s and millis to seconds
    distance = (deltaTime/1000.0)*(moveSpeed*0.215);
    //Calculate coordinates
    calculateCoords(distance);
    isMovingForward = false;
    isMovingBackward = false;
  }
}

/**
 * \par Description
 *    This function used to get the value of power supply
 * \return
 *    The power vlaue(unit is V)
 */
double get_power(void)
{
  double power;
  auriga_power = analogRead(POWER_PORT);
  power = (auriga_power/1024.0) * 15.0;
  return power;
}

void checkObstacle(void)
{
    /*Serial.print("Distance: ");
    Serial.print(ultraSensor.distanceCm());
    Serial.println(" cm");*/
    delay(100);

    if(ultraSensor.distanceCm() < 5){
      Stop();
      Backward();
      delay(500);
      randomLeftOrRight(1750); // turn for 1750ms
    }
}

void getBluetoothData(void)
{
  while(Serial.available() > 0)
  {
    for(int i = 0; i < 2; i++){
      inputArr[i] = Serial.read();
      delay(100);
    }
  }
}

void setManualDriveCommands(int input){
  //If a new command is same as the old, do nothing
  if(oldInput == input)
    return;
  else
  {
    switch (input){
        case 0:
          Stop();
          oldInput = 0;        
          break;
        case 1:
          Forward();
          oldInput = 1;
          break;
        case 2:
          Stop();
          SpinLeft();
          oldInput = 2;
          break;
        case 3:
          Stop();
          SpinRight();
          oldInput = 3;    
          break;
        case 4:
          Backward();
          oldInput = 4;
          break;
        default:
          break;
      }
  }
}


void setAutonomDriveCommands(int input){
  switch (input){
      case 0:
        Stop();        
        break;
      case 1:
        Forward();
        break;
      case 2:
        SpinLeft();
        break;
      case 3:
        SpinRight();    
        break;
      case 4:
        Backward();
        break;
      default:
        break;
    }
}

void lineFollower(void){
  if((0?(3==0?line.readSensors()==0:(line.readSensors() & 3)==3):(3==0?line.readSensors()==3:(line.readSensors() & 3)==0))){
    //Serial.println("UTE"); // SENSORS IDENTIFIES THE BLACK LINE
    Stop();
    int randomDelay = random(200, 1000);
    Backward();
    delay(500);
    randomLeftOrRight(randomDelay); // turns left or right for 500 ms
  }
  else{
    //Serial.println("INNE"); // SENSORS IDENTIFIES WHITE
    Forward();
  }
}

void sendBluetoothData(double protocol, double data1, double data2)
{
  //Fill outputArr with data
  outputArr[0] = protocol;
  outputArr[1] = data1;
  outputArr[2] = data2;
  //Send outputArr over bluetooth
  Serial.write(outputArr, 3);
  //delay(100);
}

void angleCalculation(void)
{
  //Get new gyro angle
  gyro.update();
  //Invert angle value from gyro
  newAngle = gyro.getAngleZ()*(-1.0);
  //If angle is negative, add 360 to it.
  if(newAngle < 0)
  {
    newAngle = newAngle+360;
  }
  //Add angle value to total angle, this to keep track of origo
  totalAngle = totalAngle - newAngle;
}

void calculateCoords(double distance)
{
  //Calculate X by using cosf and converting degrees to rads
  X = distance*cos(((totalAngle)*71.0)/4068.0);
  //Calculate Y by using sinf and converting degrees to rads
  Y = distance*sin(((totalAngle)*71.0)/4068.0);
  
  if(isMovingForward == true && isMovingBackward == false)
  {
    //Send positive (forwards) X and Y values
    sendBluetoothData(2, X, Y);
  }
  else if(isMovingBackward == true && isMovingBackward == true)
  {
    //Send negative (backwards) X and Y values
    sendBluetoothData(2, -X, -Y);
  }
}

// delayInput decides the active turn time, the longer the delay the longer it will rotate
void randomLeftOrRight(int16_t delayInput)
{
  randomNumber = random(0, 20);
  if(randomNumber < 10){
    SpinLeft();
    delay(delayInput);
  }
  else{
    SpinRight();
    delay(delayInput);  
  }
}

void loop() // put your main code here, to run repeatedly:
{ 
  //Update inputArr with new BT data
  getBluetoothData(); 
  //Set drive commands based on data from inputArr
  setManualDriveCommands(inputArr[1]);

  //sendBluetoothData();
  /*
  if(inputArr[1] == 2 || inputArr[1] == 3)
    angleCalculation();
      
  //sendCoord(2, 100);
  /*if(inputArr[0] == 1){ //Manual mode
    getDriveCommands();
    setDriveCommands(inputArr[1]);
  }
  else if(inputArr[0] == 0){ //Auto mode
   
  }
  else{ //this should never happen
    //Serial.println("invalid state");
  }*/
  //inputArr[8] = get_power();
 // Serial.println(0-inputArr[8]);
}
