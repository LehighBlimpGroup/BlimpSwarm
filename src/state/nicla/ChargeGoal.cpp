


#include "state/nicla/NiclaState.h"

RobotState* ChargeGoal::statetransitions(float sensors[], float controls[]) {
    if (controls[0] < 2) {
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    } else {
        if (hist->nicla_flag & 0x80) {
            if (millis() - charge_timer > 15000) {
                // hist->nicla_flag = 0x40; // switch to balloon mode
                hist->z_estimator = sensors[1];
                RobotState* levyWalk = new LevyWalk();
                return levyWalk;
            } else {
                return this;
            }
        } else if (hist->nicla_flag & 0x40) {
            if (millis() - charge_timer > 2000) {
                hist->num_captures += 1;
                if (hist->num_captures >= terms.num_captures || millis() - hist->start_ball_time > terms.time_in_ball * 1000) {
                    hist->num_captures = 0;
                    hist->nicla_desired = 1;
                    hist->start_ball_time = millis();
                }
                
                hist->z_estimator = sensors[1];
                RobotState* levyWalk = new LevyWalk();
                return levyWalk;
            } else {
                return this;
            }
        } else {
            return this;
        }
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


