


#include "state/nicla/NiclaState.h"

RobotState* LevyWalk::statetransitions(float sensors[], float controls[]) {
    if (controls[0] != 2){
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (detected(sensors)) {
        hist.z_estimator = sensors[1];
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    }
    else {
        return this; //pointer to itself
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void LevyWalk::behavior(float sensors[], float controls[], float outControls[]) {
    float _yaw = sensors[5];    
    if (millis() - levyTimer > levyDuration) {// checks if duration for current yaw is over
        levyTimer = millis();
        float lambda = 1.0 / 5000.0; // Adjust lambda for scaling; 5000 is the mean of the distribution
        float randomValue = random(1, 10001) / 10000.0; // Generate a random float between 0.0001 and 1
        unsigned long duration = (unsigned long)(-log(randomValue) / lambda);
        // Ensure the duration is within the desired range (0 to 10,000 ms)
        duration = duration % 30001; // Modulo to restrict within range if necessary
        levyDuration = duration;
        levyYaw = _yaw + random(-180, 180)/180.0f * 3.14;
    } 
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = controls[2]; //fz
    outControls[3] = 0; //tx
    outControls[4] = levyYaw; //tz
}


LevyWalk::LevyWalk() : NiclaState() {
    levyDuration = 0;
    levyYaw = 0;
    levyTimer = 0;
}


