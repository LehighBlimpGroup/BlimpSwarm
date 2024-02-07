#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include <Arduino.h>


#define SERVO1 D0
#define SERVO2 D1
#define THRUST1 D9
#define THRUST2 D10
#define BATT A2

Servo servo1;
Servo servo2; 
Servo thrust1;
Servo thrust2;

uint8_t mac_addr[6] = {0xC0, 0x49, 0xEF, 0xE3, 0x34, 0x78};//C0:49:EF:E3:34:78

Robot* myRobot = nullptr;
LowLevelComm* espNowComm = nullptr;
BaseCommunicator* baseComm = nullptr;

float actuationCommands[] = {0.0, 0.0, 0.0, 0.0};
void setup() {
  Serial.begin(115200);
  espNowComm = new LLC_ESPNow();
  baseComm = new BaseCommunicator(espNowComm);
  
  Serial.println("Start!");
  // init comms
  // wait for parameters from ground station until start parameter is set
  // wait for start parameter
  // init basecommunicator settings with new parameters
    // set ping stations?
    // set ground station

  // init robot with new parameters
  myRobot = RobotFactory::createRobot("RawBicopter");

  Serial.println("Starting Motor Servo Init");
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(BATT, INPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servo1.setPeriodHertz(50); // Standard 50hz servo
  servo2.setPeriodHertz(50); // Standard 50hz servo

  int servo_min = 500;
  int servo_max = 2500;  //FIXME set it somewhere else

  servo1.attach(SERVO1, servo_min, servo_max);
  servo2.attach(SERVO2, servo_min, servo_max);
  pinMode(THRUST1, OUTPUT);
  pinMode(THRUST2, OUTPUT);

  // init sensors with new parameters

  espNowComm->init();  //fixme this should be in the communicator
    //LowLevelComm* espNowComm = new ESPNowComm();


    baseComm->setMainBaseStation(mac_addr);

}
float s1 = 0;
float s2 = 0;
float m1 = 0;
float m2 = 0;
int min_thrust = 1050;
int max_thrust = 1500;
void loop() {
  
    // Test basecomm class
    if (baseComm->readNewMessages()) {

        if (baseComm->isNewMsgCmd()){
          ControlInput cmdMsg = baseComm->receiveMsgCmd();
          Serial.print("Cmd arrived ");
          
          Serial.print(":");

          int n = sizeof(cmdMsg.params)/ sizeof(cmdMsg.params[0]);
          for(int i = 0; i < 4; ++i) {
              actuationCommands[i] = cmdMsg.params[i];
              Serial.print(cmdMsg.params[i]); // Print each byte in hexadecimal
              Serial.print(" "); // Print a space between bytes for readability
          }
          Serial.println();
        }
    }
    s1 = clamp(actuationCommands[2], 0, 180) ; // cant handle values between PI and 2PI
    s2 = clamp(actuationCommands[3], 0, 180) ;
    
    servo1.write((int)(s1 ));
    servo2.write((int)((180 - s2)));
    m1 = clamp(actuationCommands[0], 0, 1);
    m2 = clamp(actuationCommands[1], 0, 1);
    thrust1.writeMicroseconds((int)((m1) * (max_thrust - min_thrust) + min_thrust));
    thrust2.writeMicroseconds((int)((m2) * (max_thrust - min_thrust) + min_thrust));
    //myRobot->actuate(actuationCommands, sizeof(actuationCommands) / sizeof(actuationCommands[0]));
    sleep(.1);
}

float clamp(float in, float min, float max)
{
  if (in < min)
  {
    return min;
  }
  else if (in > max)
  {
    return max;
  }
  else
  {
    return in;
  }
}