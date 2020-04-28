// INCLUDES
#include <Arduino.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// DECLARATIONS
  MeBuzzer buzzer;
  MeUltrasonicSensor ultraSensor(PORT_10);
  MeLineFollower line(PORT_9);
  MeEncoderOnBoard Encoder_1(SLOT1);
  MeEncoderOnBoard Encoder_2(SLOT2);
  MeEncoderMotor encoders[2];
  MeBluetooth bluetooth(PORT_3);

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
  float get_power(void);
  void lineFollower(void);
  void getManualInstructions(void);
  void driveCommands(int input);
  void checkObstacle(void);
  
// VARIABLES
int16_t moveSpeed = 200;
int16_t auriga_power = 0;
byte inputArr[8] = {0};

// DEFINES
#define POWER_PORT  A4

void setup()  // put your setup code here, to run once:
{
  delay(5);
  Serial.begin(115200);
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
    Encoder_1.pulsePosPlus();;
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
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward.
 */
void Backward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit to turn left.
 */
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed/2);
}

/**
 * \par Description
 *    This function use to control the car kit to turn right.
 */
void TurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed/2);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward and turn left(fast).
 */
void SpinLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Description
 *    This function use to control the car kit go backward and turn right(fast).
 */
void SpinRight(void)
{
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
}

/**
 * \par Description
 *    This function used to get the value of power supply
 * \return
 *    The power vlaue(unit is V)
 */
float get_power(void)
{
  float power;
  auriga_power = analogRead(POWER_PORT);
  power = (auriga_power/1024.0) * 15;
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
      buzzer.tone(262, 0.25 * 1000);
      
      Backward();
      delay(500);
      SpinLeft();
      delay(1750);
    }
}

void getManualInstructions(void)
{

  while(Serial.available() > 0)
  {  
    inputArr[0] = Serial.read();
    //inputArr[0] = Serial.parseInt();
    Serial.println(inputArr[0]);
    int temp = Serial.parseInt();
    delay(100);
  }
}

void driveCommands(int input){
  switch (input){
      case 0:
        Stop();
        break;
      case 1:
        Forward();
        break;
      case 2:
        TurnLeft();
        break;
      case 3:
        TurnRight();
        break;
      case 4:
        SpinLeft();
        break;
      case 5:
        SpinRight();
        break;
      case 6:
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

    Backward();
    delay(500);
    SpinLeft();
    delay(500);
  }
  else{
    //Serial.println("INNE"); // SENSORS IDENTIFIES WHITE
    Forward();
  }
}
void loop() // put your main code here, to run repeatedly:
{ 
  getManualInstructions();
  driveCommands(inputArr[0]);
  inputArr[1] = get_power();
  //checkObstacle();
  //lineFollower();
  Serial.println(inputArr[1]);
}
