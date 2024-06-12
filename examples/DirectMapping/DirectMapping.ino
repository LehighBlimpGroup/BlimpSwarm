/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
// #include "robot/RobotFactory.h"
#include "robot/RawBicopter.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/Barometer.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// MAC of the base station
uint8_t base_mac[6] = {0xC0, 0x49, 0xEF, 0xE3, 0x34, 0x78};  // fixme load this from memory


BarometerOld baro;

const float TIME_STEP = .004;
// Robot
Robot* myRobot = nullptr;
// Communication
BaseCommunicator* baseComm = nullptr;
// Control input from base station
ControlInput cmd;
ReceivedData rcv; 

float estimatedZ = 0;
float startHeight = 0;
float estimatedVZ = 0;
float kpz = 0;
float kdz = 0;

Preferences preferences; //initialize the preferences 
bool updateParams = true;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");

    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("RawBicopter");

    paramUpdate();
}


void loop() {

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


    /**
     * Begin of Controller for Height:
     * Create your height PID to control m1 and m2 here.
     * kpz and kdz are changed from the ground station and are floats declared at the top
     */
    float m1 = 0; 
    float m2 = 0;  
    float s1 = 0;
    float s2 = 0;
    m1 = cmd.params[0]*0.5 + 0.5;
    m2 = cmd.params[1]*0.5 + 0.5;
    s1 = cmd.params[2]*90 + 90;
    s2 = cmd.params[2]*90 + 90;
    
    /**
     * End of controller
     */
    // Control input
    ControlInput actuate;
    actuate.params[0] = m1; // Motor 1
    actuate.params[1] = m2; // Motor 2
    // servo control
    actuate.params[2] = s1; // Servo 1
    actuate.params[3] = s2; // Servo 2
    actuate.params[4] = cmd.params[4]; //led
    // Send command to the actuators
    myRobot->actuate(actuate.params, 5);

    sleep(TIME_STEP);
}


void paramUpdate(){
    preferences.begin("params", true); //true means read-only

    kpz = preferences.getFloat("kpz", .2); //(value is an int) (default_value is manually set)
    kdz = preferences.getFloat("kdz", 0); //(value is an int) (default_value is manually set)
    baseComm->setMainBaseStation();
    

    Serial.print("Update Paramters!");
    Serial.print(kpz);
    Serial.print(", ");
    Serial.println(kdz);
    preferences.end();
}