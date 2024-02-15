/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

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

    // Start sensor
    baro.init();
    baro.updateBarometer();
    estimatedZ = baro.getEstimatedZ();
    startHeight = baro.getEstimatedZ();
    estimatedVZ = baro.getVelocityZ();
}


void loop() {

    if (baseComm->isNewMsgCmd()){
      // New command received
      cmd = baseComm->receiveMsgCmd();

      // Print command
      Serial.print("Cmd arrived: ");
      printControlInput(cmd);
    }

    // Update measurements
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


    /**
     * Begin of Controller for Height:
     * Create your height PID to control m1 and m2 here.
     */
    float desired_height = cmd.params[0];
    float m1 = 0;  // YOUR INPUT FOR THE MOTOR 1 HERE
    float m2 = 0;  // YOUR INPUT FOR THE MOTOR 1 HERE

    /**
     * End of controller
     */
    // Control input
    ControlInput actuate;
    actuate.params[0] = m1; // Motor 1
    actuate.params[1] = m2; // Motor 2
    // servo control
    actuate.params[2] = cmd.params[2]; // Servo 1
    actuate.params[3] = cmd.params[3]; // Servo 2
    actuate.params[4] = cmd.params[4]; //led
    // Send command to the actuators
    myRobot->actuate(actuate.params, 5);

    sleep(TIME_STEP);
}



