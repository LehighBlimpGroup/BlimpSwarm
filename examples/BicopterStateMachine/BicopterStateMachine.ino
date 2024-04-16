/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "state/nicla/NiclaState.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Robot
FullBicopter* myRobot = nullptr;

// Sensor
NiclaSuite* nicla = nullptr;

// Behavior
RobotStateMachine* stateMachine = nullptr;
nicla_t terms; 
hist_t* hist;

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput behave;
ControlInput cmd;
ReceivedData rcv; 

// Data storage for the sensors 
float senses[myRobot->MAX_SENSORS];

bool updateParams = true;
const int TIME_STEP_MICRO = 4000;

int niclaOffset = 11;
int dt = 1000;
unsigned long clockTime;
unsigned long printTime;

// int nicla_flag = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = new FullBicopter();//RobotFactory::createRobot("FullBicopter");
    myRobot->startup();

    nicla = &(myRobot->sensorsuite);
    stateMachine = new RobotStateMachine(new LevyWalk());
    hist = NiclaConfig::getInstance()->getDynamicHistory();
    paramUpdate();

    nicla->changeNiclaMode(0x80);

    // updates the ground altitude for the ground feedback
    // TODO: make some way to access the actual ground height from robot
    int numSenses = myRobot->sense(senses);
    
}


void loop() {
  // Retrieves cmd.params from ground station and checks flags
  recieveCommands();

  // Get sensor values
  int numSenses = myRobot->sense(senses);
  
  // send values to ground station
  rcv.flag = 1;
  rcv.values[0] = senses[1];  //height
  rcv.values[1] = senses[5];  //yaw
  rcv.values[2] = senses[16];  //x
  rcv.values[3] = senses[17];  //y
  rcv.values[4] = senses[18];  //w
  rcv.values[5] = senses[19];  //h
  bool sent = baseComm->sendMeasurements(&rcv);

  // print sensor values every second
  // senses => [temperature, altitude, veloctity in altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, null, battery]
  if (micros() - printTime > 515106){
      Serial.print(dt/1000.0f);
      Serial.print(",");
    for (int i = 0; i < numSenses-1; i++){
      Serial.print(senses[i]);
      Serial.print(",");
    }
    Serial.println(senses[numSenses-1]);
    printTime = micros();
  }


  // adjusts the state based on several factors
  niclaStateChange((int)(cmd.params[0]));

  // Create Behavior based on sensory input to put into behave
  stateMachine->update(senses, cmd.params, behave.params);

  // Send command to the actuators
  myRobot->control(senses, behave.params, 5);

  // makes the clock rate of the loop consistant. 
  fixClockRate();
}

void recieveCommands(){
  if (baseComm->isNewMsgCmd()){
    // New command received
    cmd = baseComm->receiveMsgCmd();
    if (int(cmd.params[11]) == 1 && updateParams){
      paramUpdate();
      updateParams = false;
    } else {
      updateParams = true;
    }
    // Print command
    Serial.print("Cmd arrived: ");
    printControlInput(cmd);
  }
}

void paramUpdate(){

    myRobot->getPreferences();
    baseComm->setMainBaseStation();
    NiclaConfig::getInstance()->loadConfiguration();
    
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    
}

void niclaStateChange(int cmdFlag) {
  int detected = 0;
  int nicla_flag = (int)senses[niclaOffset + 0];
  float tracking_x = (float)senses[niclaOffset + 1];
  float tracking_y =(float)senses[niclaOffset + 2];
  float detection_w = (float)senses[niclaOffset + 7];
  float detection_h = (float)senses[niclaOffset + 8];

  if ((nicla_flag & 0b11000000 == 0) && (hist->nicla_flag & 0b11000000)){
      // old flag indicated a detection while the new flag says no detection
      // that is a negative edge
      detected = -1;
  } else if (nicla_flag & 0x40) {
      // balloon mode
      if (nicla_flag & 0b11 != hist->nicla_flag & 0b11) {
          // the last two MSBs of the flag toggles between 0b01 and 0b10 for new detections,
          // and it toggles to 0b00 for new no-detection
          detected = 1;
      }
  } else if (nicla_flag & 0x80) {
      // goal mode
      if (hist->last_tracking_x != tracking_x || hist->last_tracking_y != tracking_y || hist->last_detection_w != detection_w || hist->last_detection_h != detection_h) {
          detected = 1;
      }
  }
  if (detected) { // positive edge to avoid spamming
    int hist_flag = hist->nicla_flag;
    if (cmdFlag == 2) {// normal state machine mode
      if (hist->nicla_flag & 0x80) {
        if (nicla_flag & 0x40) {
          nicla->changeNiclaMode(0x80);
        }
      } 
      else if (hist->nicla_flag & 0x40) {
        if (nicla_flag & 0x80) {
          nicla->changeNiclaMode(0x40);
        }
      }
    } 
    else if (cmdFlag == 3) { //balloon only mode (enforce 0x40)
      if (hist->nicla_flag & 0x80) {
        hist->nicla_flag = 0x40;
        NiclaConfig::getInstance()->loadConfiguration();
      }
      if (nicla_flag & 0x80) {
        nicla->changeNiclaMode(0x40);
      }
    } 
    else if (cmdFlag == 4) { //goal only mode (enforce 0x80)
      if (hist->nicla_flag & 0x40) {
        hist->nicla_flag = 0x80;
        NiclaConfig::getInstance()->loadConfiguration();
      }
      if (nicla_flag & 0x80) {
        nicla->changeNiclaMode(0x40);
      }
    }
  }
  
}

void fixClockRate() {

  dt = (int)(micros()-clockTime);
  while (TIME_STEP_MICRO - dt > 0){
    dt = (int)(micros()-clockTime);
  }
  clockTime = micros();
}
