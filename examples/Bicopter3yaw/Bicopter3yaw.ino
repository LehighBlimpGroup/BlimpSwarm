/**
 * BICOPTER with altitude and yaw control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Robot
Robot* myRobot = nullptr;

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput cmd;
ControlInput outputs;
ReceivedData rcv; 

// Data storage for the sensors 
float senses[myRobot->MAX_SENSORS];

bool updateParams = true;
const int TIME_STEP_MICRO = 4000;
float groundAltitude = 0;
int dt = 1000;
unsigned long clockTime;
unsigned long printTime;


typedef struct feedback_s {
    bool zEn, yawEn;
    float kpyaw, kdyaw, kiyaw;
    float kpz, kdz, kiz, z_int_low, z_int_high;
} feedback_t;

feedback_t PDterms;

// List of the variables that need persistent storage
float z_integral = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("FullBicopter");
    paramUpdate();

    // updates the ground altitude for the ground feedback
    int numSenses = myRobot->sense(senses);
    groundAltitude = senses[1];//height
}


void loop() {
  // Retrieves cmd.params from ground station and checks flags
  recieveCommands();

  // Get sensor values
  int numSenses = myRobot->sense(senses);
  
  // send values to ground station
  rcv.flag = 1;
  rcv.values[0] = senses[1] - groundAltitude;  //height
  rcv.values[1] = senses[5];  //yaw
  rcv.values[2] = senses[10];  //battery
  rcv.values[3] = senses[0];  //temperature
  bool sent = baseComm->sendMeasurements(&rcv);

  // print sensor values every second
  // senses => [temperature, altitude, veloctity in altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, null, battery]
  if (micros() - printTime > 500000){
    for (int i = 0; i < numSenses-11; i++){
      Serial.print(senses[i]);
      Serial.print(",");
    }
    Serial.println(senses[numSenses-11]);
    printTime = micros();
  }

 
  // Send command to the actuators
  
  myRobot->control(senses, cmd.params, 5);

  // makes the clock rate of the loop consistent.
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
    Preferences preferences; //initialize the preferences 
    preferences.begin("params", true); //true means read-only

    // PDterms.zEn = preferences.getBool("zEn", false);
    // PDterms.zEn = preferences.getBool("yawEn", false);
    // PDterms.kpz = preferences.getFloat("kpz", 0.5);
    // PDterms.kdz = preferences.getFloat("kdz", 0.5);
    // PDterms.kiz = preferences.getFloat("kiz", 0);
    // PDterms.z_int_low = preferences.getFloat("z_int_low", 0);
    // PDterms.z_int_high = preferences.getFloat("z_int_high", 0.2);
    // PDterms.kpyaw = preferences.getFloat("kpyaw", 0.1);
    // PDterms.kdyaw = preferences.getFloat("kdyaw", 0.1);// same thing as if I said kpyawrate
    // PDterms.kiyaw = preferences.getFloat("kiyaw", 0);

    preferences.end();

    myRobot->getPreferences();
    baseComm->setMainBaseStation();
}

void fixClockRate() {

  dt = (int) (micros()-clockTime);
  while (TIME_STEP_MICRO - dt > 0){
    dt = (int) (micros()-clockTime);
  }
  clockTime = micros();
}
