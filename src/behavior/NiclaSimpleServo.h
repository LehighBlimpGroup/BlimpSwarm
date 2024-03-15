//
// Created by eddie on 3/14/24.
//

#ifndef BLIMPSWARM_NICLASIMPLESERVO_H
#define BLIMPSWARM_NICLASIMPLESERVO_H


#include "BehaviorInterface.h"
#include "Manual.h"

class NiclaSimpleServo : public BehaviorInterface {
public:
    // Virtual destructor to ensure derived class destructors are called
    NiclaSimpleServo();
    float max_x = 240;
    float max_y = 160;
    float last_detection_x = 0;
    float last_detection_y = 0;
    float last_detection_w = 0;
    float last_detection_h = 0;
    float last_tracking_x = 0;
    float last_tracking_y = 0;
    float last_tracking_w = 0;
    float last_tracking_h = 0;
    float des_yaw = 0;
    float robot_to_goal = 0;

    // behave is the function that controls the behavior of the blimp
    // it returns an integer that indicates a state transistion for other behaviors to read
    int behave(float sensors[], float controls[], float outControls[]) override;

    // reads from Preferences library to initialize variables
    void getPreferences() override;

private:
    Manual manual;
    int servoing(float sensors[], float controls[], float outControls[]);
};


#endif //BLIMPSWARM_NICLASIMPLESERVO_H
