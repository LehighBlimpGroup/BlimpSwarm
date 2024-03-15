    
    
#include "NiclaSimpleServo.h"


NiclaSimpleServo::NiclaSimpleServo(){

}


int NiclaSimpleServo::servoing(float sensors[], float controls[], float outControls[]) {
    
    int niclaOffset = 11;
    int nicla_flag = (int)sensors[niclaOffset + 0];
    float tracking_x = (float)sensors[niclaOffset + 1];
    float tracking_y =(float)sensors[niclaOffset + 2];
    float tracking_w = (float)sensors[niclaOffset + 3];
    float tracking_h = (float)sensors[niclaOffset + 4];
    float detection_x =(float)sensors[niclaOffset + 5];
    float detection_y = (float)sensors[niclaOffset + 6];
    float detection_w = (float)sensors[niclaOffset + 7];
    float detection_h = (float)sensors[niclaOffset + 8];
    float _yaw = sensors[5];

    if (last_tracking_x != tracking_x || last_tracking_y != tracking_y || last_detection_w != detection_w || last_detection_h != detection_h) {
        float x_cal = tracking_x / max_x;
        float y_cal = tracking_y / max_y;
        des_yaw = ((x_cal - 0.5));
        robot_to_goal = _yaw + des_yaw;
    }

    outControls[0] = controls[0];
    outControls[1] = controls[1];
    outControls[2] = controls[2];
    outControls[3] = controls[3];
    outControls[4] = robot_to_goal;
    return 1;

}

// behave is the function that controls the behavior of the blimp
// it returns an integer that indicates a state transistion for other behaviors to read
int NiclaSimpleServo::behave(float sensors[], float controls[], float outControls[]) {
    if (controls[0] == 2){
        return servoing(sensors, controls, outControls);

    } else {
        return manual.behave(sensors, controls, outControls);
    }
    return 1;
}

// reads from Preferences library to initialize variables
void NiclaSimpleServo::getPreferences() {
    return;
}