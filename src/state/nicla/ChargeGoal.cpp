


#include "state/nicla/NiclaState.h"

RobotState* ChargeGoal::statetransitions(float sensors[], float controls[]) {
    if (controls[0] != 2) {
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (millis() - charge_timer > 10000) {
        hist->z_estimator = sensors[1];
        RobotState* levyWalk = new LevyWalk();
        return levyWalk;
    } else {
        return this;
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void ChargeGoal::behavior(float sensors[], float controls[], float outControls[]) {
    // Serial.println("charging!");
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_charge; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = hist->robot_to_goal; //tz
}


ChargeGoal::ChargeGoal() : NiclaState() {
    charge_timer = millis();
    
}


