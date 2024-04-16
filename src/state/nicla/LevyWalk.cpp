


#include "state/nicla/NiclaState.h"

RobotState* LevyWalk::statetransitions(float sensors[], float controls[]) {
    if (controls[0] < 2){
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (detected(sensors)) {
        float _yaw = sensors[5];  
        float _height = sensors[1];  
        int niclaOffset = 11;
        float tracking_x = (float)sensors[niclaOffset + 1];
        float detection_y = (float)sensors[niclaOffset + 6];
        float x_cal = tracking_x / terms.n_max_x;
        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    }
    else {
        return this; //pointer to itself
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void LevyWalk::behavior(float sensors[], float controls[], float outControls[]) {
    // Serial.println("Walking!");
    if (millis() - levyTimer > levyDuration) {// checks if duration for current yaw is over
        float _yaw = sensors[5];  
        hist->z_estimator = sensors[1] + random(-4000, 10001) / 10000.0;
        levyTimer = millis();
        float lambda = 1.0 / 5000.0; // Adjust lambda for scaling; 5000 is the mean of the distribution
        float randomValue = random(1, 10001) / 10000.0; // Generate a random float between 0.0001 and 1
        unsigned long duration = (unsigned long)(-log(randomValue) / lambda);
        // Ensure the duration is within the desired range (0 to 10,000 ms)
        duration = duration % 30001; // Modulo to restrict within range if necessary
        levyDuration = duration;
        levyYaw = _yaw + random(-180, 180)/180.0f * 3.14;
    } 
    int dt = SpiralTimer - millis();
    SpiralTimer += dt;
    if (abs(yawCurr - levyYaw) >0){
        yawCurr += (float) (1000.0f/(float)dt) * constrain(levyYaw - yawCurr, -.5, .5);
    }
    
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = levyYaw; //tz
}


LevyWalk::LevyWalk() : NiclaState() {
    SpiralTimer = millis();
    levyDuration = 2000;
    levyYaw = hist->robot_to_goal;
    yawCurr = hist->robot_to_goal;
    levyTimer = millis();
}


