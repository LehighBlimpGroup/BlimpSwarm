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
#include <KalmanFilterDrone2.h>

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

// kalman state estimator

// bool verbose = false;
const float scale = .0001;
float Q_val = 500 * scale;
float R_val = 1000000 * scale;
float P_val = 2000 * scale;
float Pv_val = 200 * scale;
const float width = 30.0 * scale;
const float height = 12.5 * scale;
State state;
float x_vel = 0;
float y_vel = 0;
float vel_gamma = .9;

float kal_x = 0;
float kal_y = 0; 
float kal_eig0 = 10; 
float kal_eig1 = 10;
float kal_eig_angle = 0;

BLA::Matrix<4, 1> x;
KalmanHandler* kalman_handler = nullptr;
long startTime = 0;



bool updateParams = true;
const int TIME_STEP_MICRO = 4000;

int niclaOffset = 11;
int dt = 1000;
unsigned long clockTime;
unsigned long printTime;
unsigned long nicla_change_time;

// int nicla_flag = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    startTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = new FullBicopter();//RobotFactory::createRobot("FullBicopter");
    myRobot->startup();

    nicla = &(myRobot->sensorsuite);
    stateMachine = new RobotStateMachine(new LevyWalk());
    
    paramUpdate();
    hist->nicla_flag = 0x80;

    // nicla->changeNiclaMode(0x80);


    // Kalman Setup
    x.Fill(0);
    x(0, 0) = 15 * scale;
    x(1, 0) = 6.24 * scale;
    kalman_handler = new KalmanHandler(x, scale, width, height);
    
    kal_x = kalman_handler->kalman->x(0) / scale;
    kal_y = kalman_handler->kalman->x(1) / scale;
        

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
  if (cmd.params[0] == 5 && cmd.params[6] == 1) { 
    rcv.flag = 1;
    rcv.values[0] = senses[1];  //height
    rcv.values[1] = senses[5];  //yaw
    rcv.values[2] = kal_x;  //x
    rcv.values[3] = kal_y;  //y
    rcv.values[4] = kal_eig_angle ;  //w
    rcv.values[5] = senses[10];  //battery
    // Serial.println("Sending Feedback.");
    bool sent = baseComm->sendMeasurements(&rcv);
    // cmd.params[0] = 1; // temp assign manual control with these new params for retaining stillness
    // cmd.params[2] += senses[1]; // set height to height
    // cmd.params[4] += senses[5]; // set yaw to yaw
  } else if(cmd.params[0] == 5 && cmd.params[6] == 0) {
    rcv.flag = 0;
    Serial.println("Stopping Feedback");
    cmd.params[0] = 1; // temp assign manual control with these new params for retaining stillness
    cmd.params[2] += senses[1]; // set height to height
    cmd.params[4] += senses[5]; // set yaw to yaw
  } else if (cmd.params[6] == 1){
    rcv.flag = 1;
    rcv.values[0] = senses[1];  //height
    rcv.values[1] = senses[5];  //yaw
    // rcv.values[2] = senses[niclaOffset + 5];  //x
    // rcv.values[3] = senses[niclaOffset + 6];  //y
    // rcv.values[4] = senses[niclaOffset + 7];  //w
    rcv.values[5] = senses[10];  //battery
    bool sent = baseComm->sendMeasurements(&rcv);

  }
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
    Serial.print("kalman: " );
    Serial.print(kal_x);
    Serial.print(",");
    Serial.println(kal_y);
    printTime = micros();
  }


  // adjusts the state based on several factors
  //niclaStateChange((int)(cmd.params[0]), (int)(cmd.params[7]));

  // Create Behavior based on sensory input to put into behave
  //stateMachine->update(senses, cmd.params, behave.params);

  // Send command to the actuators
  myRobot->control(senses, cmd.params, 5);
  
  // kalman predicted state
  kalmanUpdate();

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
    
    hist = NiclaConfig::getInstance()->getDynamicHistory();
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    
}


int kalFlag = -1;
void kalmanUpdate() {
  float dt = (micros() - startTime)/1000000.0;
  startTime = micros();
  state.angle = senses[5] *180.0f/ 3.14159f; //yaw
  state.distance_to_wall = 0;
  state.wall_id = 0;
  state.test = 0;
  if (kalFlag != senses[niclaOffset] && senses[niclaOffset] < 257) {
    state.test = 1;
  }
  kalFlag = senses[niclaOffset];

  state.shapes_in_range_count = 0;
  
  // Update shapes_ids
  for (int i = 0; i < 2; i++) {
      state.shapes_ids[i] = static_cast<int>(senses[niclaOffset + i*3 + 3]);
  }
  
  // Update angles_to_shapes
  for (int i = 0; i < 2; i++) {
      state.angles_to_shapes[i] = (senses[niclaOffset + i*3 + 1]-120)/120.0f * 30;
      if (senses[niclaOffset + i*3 + 1] != 0 && senses[niclaOffset + i*3 + 2] != 0){
        state.shapes_in_range_count += 1;
      }
  }

  BLA::Matrix<2, 1> u;
  float forward_accel = cmd.params[1]; // fx value
  float angle = senses[5];
  u.Fill(0);
  x_vel = x_vel * vel_gamma - forward_accel * sin(angle) * (1-vel_gamma);
  y_vel = y_vel * vel_gamma - forward_accel * cos(angle) * (1-vel_gamma);
  u(0, 0) =  x_vel * scale;
  u(1, 0) = y_vel * scale;
  kalman_handler->update(state, u, dt);



  kal_x = kalman_handler->kalman->x(0) / scale;
  kal_y = kalman_handler->kalman->x(1) / scale;

  kal_eig0 = kalman_handler->eigenvalue0 / scale;
  kal_eig1 = kalman_handler->eigenvalue1 / scale;
  kal_eig_angle = kalman_handler->angle;
  


}


void niclaStateChange(int cmdFlag, int target_color) {

  int nicla_flag = senses[niclaOffset + 0];
  if (micros() - nicla_change_time > 50000) { // positive edge to avoid spamming
    nicla_change_time = micros();
    int hist_flag = hist->nicla_flag;
    if (cmdFlag == 2) { // normal state machine mode
      if (hist->nicla_desired == 1) {
        if (nicla_flag & 0x40) {
          if (target_color == 1) {
            Serial.println("go to goal");
            nicla->changeNiclaMode(0x81);

          } else {
            Serial.println("go to goal");
            nicla->changeNiclaMode(0x80);
          }
          hist->z_estimator = terms.goal_height;
        }
      } 
      else if (hist->nicla_desired == 0) {
        if (nicla_flag & 0x80) {
          Serial.println("go to ball");
          nicla->changeNiclaMode(0x40);
          hist->start_ball_time= millis();
        }
      }
    } 
    else if (cmdFlag == 3) { //balloon only mode (enforce 0x40)
      hist->nicla_desired = 0;
      hist->start_ball_time= millis();
      hist->num_captures = 0;
      if (nicla_flag & 0x80) {
        Serial.println("go to ball");
        nicla->changeNiclaMode(0x40);
      }
    } 
    else if (cmdFlag == 4) { //goal only mode (enforce 0x80)
      if (hist_flag != 4) {
        hist->goal_direction = senses[5];
      }
      hist->nicla_desired = 1;
      if (nicla_flag & 0x40) {
        if (target_color == 1){
          Serial.println("go to goal");
          nicla->changeNiclaMode(0x81);

        } else {
          Serial.println("go to goal");
          nicla->changeNiclaMode(0x80);
        }
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
