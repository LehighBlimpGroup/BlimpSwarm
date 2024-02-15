#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/Barometer.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// MAC of the base station
uint8_t base_mac[6] = {0xC0, 0x49, 0xEF, 0xE3, 0x34, 0x78};  // fixme load this from memory


Barometer baro;

const float TIME_STEP = .01;
// Robot
Robot* myRobot = nullptr;
// Communication
BaseCommunicator* baseComm = nullptr;
// Control input from base station
ControlInput cmd;

float estimatedZ = 0;
float startHeight = 0;
float estimatedVZ = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");

    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation(base_mac);

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("RawBicopter");
    baro.init();
    baro.updateBarometer();
    estimatedZ = baro.getEstimatedZ();
    startHeight = baro.getEstimatedZ();
    estimatedVZ = baro.getVelocityZ();
    
    // TODO full logic
    // wait for parameters from ground station until start parameter is set
    // wait for start parameter
    // init basecommunicator settings with new parameters
    // set ping stations?
    // set ground station





}


void loop() {

    if (baseComm->isNewMsgCmd()){
      // New command received
      cmd = baseComm->receiveMsgCmd();

      // Print command
      Serial.print("Cmd arrived: ");
      printControlInput(cmd);

    }

    if (baro.updateBarometer()){
      // sense 
      float height = baro.getEstimatedZ() - startHeight;
      float height_velocity = baro.getVelocityZ();
      // estimate
      estimatedZ = estimatedZ * .6 + height * .4;
      estimatedVZ = estimatedVZ * .90 + height_velocity * .1;
      
      Serial.print(estimatedZ);
      Serial.print(", ");
      Serial.println(estimatedVZ);
    }
    // height control
    // Create your height PID to control m1 and m2 here
    
    float desired_height = cmd.params[0];
    float m1 = 0;
    float m2 = 0;
    ControlInput actuate; //do not overwrite cmd, since it does not reupdate every loop; instead make a new object.
    
    actuate.params[0] = m1; //m1
    actuate.params[1] = m2; //m2
    // servo control
    actuate.params[2] = cmd.params[2]; //m1
    actuate.params[3] = cmd.params[3]; //m2
    actuate.params[4] = cmd.params[4]; //led
    // Send command to the actuators
    myRobot->actuate(actuate.params, 5);

    sleep(TIME_STEP);
}



