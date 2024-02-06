#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"

RobotFactory robotFactory;
Robot *robot;
BaseCommunicator comms;

void setup() {
  // init comms
  // wait for parameters from ground station until start parameter is set
  // wait for start parameter
  // init basecommunicator settings with new parameters
    // set ping stations?
    // set ground station

  // init robot with new parameters
  robot = robotFactory.createRobot("RawBicopter");
  // init sensors with new parameters

  

}

void loop() {
  //Read sensors
  //Read control data
  //Autonomous If statement
    //State machine or manual controls
      //default to manual for this version of rawbicopter
  //Convert commands to executables
  //Execute actuations
  //Send feedback and delay for clock rate

}
