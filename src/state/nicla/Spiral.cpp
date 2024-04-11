


#include "state/nicla/NiclaState.h"

RobotState* Spiral::statetransitions(float sensors[], float controls[]) {
    if (controls[0] != 2){
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
        float y_cal = detection_y / terms.n_max_y;
        if ( abs(x_cal - 0.5) < .16) { // makes sure yaw is in center before making height adjustments
            // z_offset += 20*((y_cal - 0.5))* subdt / sideLength;//(.75 - max(detection_h, detection_w)/max_y);
            hist->z_estimator =  ( _height + terms.y_strength * (y_cal - terms.y_thresh)) ; // integral must be on
        }
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    }
    else {
        return this; //pointer to itself
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void Spiral::behavior(float sensors[], float controls[], float outControls[]) {
    // Serial.println("Walking!");
    int dt = SpiralTimer - millis();
    SpiralTimer += dt;
    SpiralYaw += (float) (1000/dt) * 0.3;
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = SpiralYaw; //tz
}


Spiral::Spiral() : NiclaState() {
    SpiralTimer = millis();
}


