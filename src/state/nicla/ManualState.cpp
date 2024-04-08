


#include "state/nicla/NiclaState.h"

RobotState* ManualState::statetransitions(float sensors[], float controls[]) {
    if (controls[0] != 2){
        hist->z_estimator = sensors[1];
        return this;
    }
    else if (detected(sensors)) {
        hist->z_estimator = sensors[1];
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    }
    else {
        hist->z_estimator = sensors[1];
        RobotState* levyWalk = new LevyWalk();
        return levyWalk;
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void ManualState::behavior(float sensors[], float controls[], float outControls[]) {
    
    outControls[0] = controls[0]; //ready
    outControls[1] = controls[1]; //fx
    outControls[2] = controls[2]; //fz
    outControls[3] = controls[3]; //tx
    outControls[4] = controls[4]; //tz
}


ManualState::ManualState() : NiclaState() {
    
}


