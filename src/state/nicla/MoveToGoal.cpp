

#include "state/nicla/NiclaState.h"

    
RobotState* MoveToGoal::statetransitions(float sensors[], float controls[]) {
    int niclaOffset = 11;
    int nicla_flag = (int)sensors[niclaOffset + 0];
    if (controls[0] < 2){
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (hist->nicla_flag & 0x80) {
        if (start == true){
            return this;
        }
        else if (!(nicla_flag & 0b11)) { // no detection registered by nicla in flag
            RobotState* levyWalk = new LevyWalk();
            return levyWalk;
        }
        else if (closeToGoal(sensors) ) {
            RobotState* chargeGoal = new ChargeGoal();
            return chargeGoal;
        }
        else {
            return this; //pointer to itself
        }
    } else if (hist->nicla_flag & 0x40) {
        // state transition for balloon detection
        if (!(nicla_flag & 0b11)) { // no detection registered by nicla in flag
            RobotState* levyWalk = new LevyWalk();
            return levyWalk;
        }
        else if (closeToGoal(sensors) ) {
            RobotState* chargeGoal = new ChargeGoal();
            return chargeGoal;
        }
        else {
            return this; //pointer to itself
        }
    } else {
        start = true;
        // no detection at all
        // RobotState* levyWalk = new LevyWalk();
        // return levyWalk;
        return this;
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void MoveToGoal::behavior(float sensors[], float controls[], float outControls[]) {
    int niclaOffset = 11;
    int edge = detected(sensors);
    if (start){
        
        float _yaw = sensors[5];
        
        hist->robot_to_goal = _yaw;
        hist->forward_force = 0;
    }
    if (edge == 1) {
        start = false;
        // if a new detection is fed in
        float _yaw = sensors[5];
        float _height = sensors[1];
        int nicla_flag = (int)sensors[niclaOffset + 0];
        float tracking_x = (float)sensors[niclaOffset + 1];
        float tracking_y = (float)sensors[niclaOffset + 2];
        float detection_y = (float)sensors[niclaOffset + 6];
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixles into a value between [0,1]

        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
        float y_cal = detection_y / terms.n_max_y;
        if ( abs(x_cal - 0.5) < terms.range_for_forward) { // makes sure yaw is in center before making height adjustments
            // z_offset += 20*((y_cal - 0.5))* subdt / sideLength;//(.75 - max(detection_h, detection_w)/max_y);
            hist->z_estimator =  ( _height + constrain(terms.y_strength * (y_cal - terms.y_thresh), -.5, .5)) ; // integral must be on
            hist->forward_force = terms.fx_togoal;
        } else if (nicla_flag & 0x40) {
            // balloon case
            hist->forward_force = 0.0;
        }
    }
    outControls[0] = controls[0]; //ready
    outControls[1] = hist->forward_force; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = hist->robot_to_goal; //tz
}


MoveToGoal::MoveToGoal() : NiclaState() {
    hist->forward_force = 0.0;
    start = true;
}
    
