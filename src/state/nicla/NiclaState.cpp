

#include "state/nicla/NiclaState.h"


/*
This file contains the 'structure' of the nicla state; anyt funciton that needs to be called by any state, 
as well as organizing the state parameters and history values from the NiclaConfig.h file.
*/


// function which detects the positive edge of a new image
bool NiclaState::detected(float sensors[]) {
    int niclaOffset = 11;
    bool detected = false;
    int nicla_flag = (int)sensors[niclaOffset + 0];
    float tracking_x = (float)sensors[niclaOffset + 1];
    float tracking_y =(float)sensors[niclaOffset + 2];
    float detection_w = (float)sensors[niclaOffset + 7];
    float detection_h = (float)sensors[niclaOffset + 8];
    if (hist->last_tracking_x != tracking_x || hist->last_tracking_y != tracking_y || hist->last_detection_w != detection_w || hist->last_detection_h != detection_h) {
        detected = true;
    }
    hist->last_tracking_x = (float)sensors[niclaOffset + 1];
    hist->last_tracking_y =(float)sensors[niclaOffset + 2];
    hist->last_detection_w = (float)sensors[niclaOffset + 7];
    hist->last_detection_h = (float)sensors[niclaOffset + 8];
    return detected;
}

// function which detects the positive edge of a new image
bool NiclaState::closeToGoal(float sensors[]) {
    int niclaOffset = 11;
    
    
    float detection_w = (float)sensors[niclaOffset + 7];
    float detection_h = (float)sensors[niclaOffset + 8];

    float sideLength = max(detection_h, detection_w);
    bool too_close = true;
    // if the height of the goal is less than 75% of the height, then it is not too close; 
    if (sideLength < terms.h_ratio * (float)(terms.n_max_y)) {
        too_close = false;
    }
    return too_close;
}

// gathers the config file data
NiclaState::NiclaState() {
    // Use NiclaConfig to access configuration data
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    hist = NiclaConfig::getInstance()->getDynamicHistory();
    
}

// function to update state based on sensors and controls which is called by the state machine
RobotState* NiclaState::update(float sensors[], float controls[], float outControls[]) {
    
    behavior(sensors, controls, outControls);

    return statetransitions(sensors, controls);
}
    


