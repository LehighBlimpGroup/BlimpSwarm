

#include "state/nicla/NiclaState.h"

    
RobotState* MoveToGoal::statetransitions(float sensors[], float controls[]) {
    if (controls[0] != 2){
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (detected(sensors) && closeToGoal(sensors)) {
        RobotState* chargeGoal = new ChargeGoal();
        return chargeGoal;
    }
    else if (!detected(sensors)) {
        RobotState* levyWalk = new LevyWalk();
        return levyWalk;
    }
    else {
        return this; //pointer to itself
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void MoveToGoal::behavior(float sensors[], float controls[], float outControls[]) {
    float _yaw = sensors[5];  
    float _height = sensors[1];  
    int niclaOffset = 11;
    float tracking_x = (float)sensors[niclaOffset + 1];
    float detection_y = (float)sensors[niclaOffset + 6];
    if (detected(sensors)) {// checks if new yaw occurs 
        float x_cal = tracking_x / terms.n_max_x;
        hist.des_yaw = ((x_cal - 0.5));
        hist.robot_to_goal = _yaw + hist.des_yaw;
        float y_cal = detection_y / terms.n_max_y;
        if ( abs(hist.des_yaw) < .1) { // makes sure yaw is in center before making height adjustments
            // z_offset += 20*((y_cal - 0.5))* subdt / sideLength;//(.75 - max(detection_h, detection_w)/max_y);
            hist.z_estimator =  ( _height + terms.y_strength * (y_cal - terms.y_thresh)) ; // integral must be on
        }
    } 
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_togoal; //fx
    outControls[2] = hist.z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = hist.robot_to_goal; //tz
}


MoveToGoal::MoveToGoal() : NiclaState() {
}
    
