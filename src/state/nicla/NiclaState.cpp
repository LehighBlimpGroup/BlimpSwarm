

#include "state/nicla/NiclaState.h"


/*
This file contains the 'structure' of the nicla state; anyt funciton that needs to be called by any state, 
as well as organizing the state parameters and history values from the NiclaConfig.h file.
*/


// function which detects the positive and negative edges of a new image
int NiclaState::detected(float sensors[]) {
    int niclaOffset = 11;
    int detected = 0;
    int nicla_flag = (int)sensors[niclaOffset + 0];
    float tracking_x = (float)sensors[niclaOffset + 1];
    float tracking_y =(float)sensors[niclaOffset + 2];
    float detection_w = (float)sensors[niclaOffset + 7];
    float detection_h = (float)sensors[niclaOffset + 8];

    if ((nicla_flag & 0b11000000 == 0) && (hist->nicla_flag & 0b11000000)){
        // old flag indicated a detection while the new flag says no detection
        // that is a negative edge
        detected = -1;
    } else if (hist->nicla_flag & 0x40) {
        // balloon mode
        if (nicla_flag & 0b11 != hist->nicla_flag & 0b11) {
            // the last two MSBs of the flag toggles between 0b01 and 0b10 for new detections,
            // and it toggles to 0b00 for new no-detection
            detected = 1;
        }
    } else if (hist->nicla_flag & 0x80) {
        // goal mode
        if (hist->last_tracking_x != tracking_x || hist->last_tracking_y != tracking_y || hist->last_detection_w != detection_w || hist->last_detection_h != detection_h) {
            detected = 1;
        }
    }
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
    hist = NiclaConfig::getInstance()->getDynamicHistory();
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    
}

// function to update state based on sensors and controls which is called by the state machine
RobotState* NiclaState::update(float sensors[], float controls[], float outControls[]) {
    int niclaOffset = 11;
    behavior(sensors, controls, outControls);
    RobotState* ptr = statetransitions(sensors, controls);
    hist->last_tracking_x = (float)sensors[niclaOffset + 1];
    hist->last_tracking_y =(float)sensors[niclaOffset + 2];
    hist->last_detection_w = (float)sensors[niclaOffset + 7];
    hist->last_detection_h = (float)sensors[niclaOffset + 8];
    hist->nicla_flag = (int)sensors[niclaOffset + 0];

    return ptr;
}
    


