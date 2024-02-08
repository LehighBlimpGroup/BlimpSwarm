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

// Control input from base station
ControlInput cmd;


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

  int servo_min = 550;
  int servo_max = 2450;  //FIXME set it somewhere else

  servo1.attach(SERVO1, servo_min, servo_max);
  servo2.attach(SERVO2, servo_min, servo_max);
  pinMode(THRUST1, OUTPUT);
  pinMode(THRUST2, OUTPUT);
  thrust1.attach(THRUST1, 1000, 2000);
  thrust2.attach(THRUST2, 1000, 2000);
  thrust1.setPeriodHertz(55);
  thrust2.setPeriodHertz(58);
  escarm(thrust1, thrust2);
  // User LED
  pinMode(LED_BUILTIN, OUTPUT);

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
          // New command received
          cmd = baseComm->receiveMsgCmd();


          // Print command
          Serial.print("Cmd arrived ");          
          Serial.print(":");
          int n = sizeof(cmd.params)/ sizeof(cmd.params[0]);
          for(int i = 0; i < 5; ++i) {              
              Serial.print(cmd.params[i]); // Print each byte in hexadecimal
              Serial.print(" "); // Print a space between bytes for readability
          }
          Serial.println();
        }
    }

    // Servo output
    s1 = constrain(cmd.params[2], 0, 180) ; // cant handle values between PI and 2PI
    s2 = constrain(cmd.params[3], 0, 180) ;    
    servo1.write((int)(s1 ));
    servo2.write((int)((180 - s2)));

    // Motor output
    m1 = constrain(cmd.params[0], 0, 1);
    m2 = constrain(cmd.params[1], 0, 1);
    thrust1.writeMicroseconds((int)((m1) * (max_thrust - min_thrust) + min_thrust));
    thrust2.writeMicroseconds((int)((m2) * (max_thrust - min_thrust) + min_thrust));

    // User LED
    int led = (cmd.params[4]<-0.5)? LOW: HIGH;
    digitalWrite(LED_BUILTIN, led);
    

    //myRobot->actuate(actuationCommands, sizeof(actuationCommands) / sizeof(actuationCommands[0]));
    sleep(.01);
}


// Enter arming sequence for ESC
void escarm(Servo &thrust1, Servo &thrust2)
{
  // ESC arming sequence for BLHeli S
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);

  // Sweep up
  for (int i = 1050; i < 1500; i++)
  {
    thrust1.writeMicroseconds(i);
    delay(3);
    thrust2.writeMicroseconds(i);
    delay(3);
  }
  // Sweep down
  for (int i = 1050; i > 1100; i--)
  {
    thrust1.writeMicroseconds(i);
    delay(3);
    thrust2.writeMicroseconds(i);
    delay(3);
  }
  // Back to minimum value
  thrust1.writeMicroseconds(1000);
  delay(100);
  thrust2.writeMicroseconds(1000);
  delay(10);
}

// Enter arming sequence for ESC
void calibrate_esc(Servo &thrust1, Servo &thrust2)
{
   delay(1000);
  Serial.println("Calibrating ESCs....");
  // ESC arming sequence for BLHeli S
  thrust1.writeMicroseconds(2000);
  delay(10);
  thrust2.writeMicroseconds(2000);
  delay(15000);


  // Back to minimum value
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);



  Serial.println("Calibration completed");
}
