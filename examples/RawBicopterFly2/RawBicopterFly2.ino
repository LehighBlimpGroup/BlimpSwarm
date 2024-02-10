#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include <Arduino.h>
#include <ESP32Servo.h>



#define BATT A2



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


  pinMode(BATT, INPUT);


  espNowComm->init();  //fixme this should be in the communicator
  baseComm->setMainBaseStation(mac_addr);

}

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

    myRobot->actuate(cmd.params, 5);


    sleep(.01);
}



