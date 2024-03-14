//
// Created by edward on 3/1/24.
//

#ifndef BLIMPSWARM_FULLBICOPTER_H
#define BLIMPSWARM_FULLBICOPTER_H



#include "RawBicopter.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>




class FullBicopter : public RawBicopter {
public:
    FullBicopter();
    int sense(float sensors[MAX_SENSORS]) override;
    bool control(float sensors[MAX_SENSORS], float controls[], int size ) override;
    void getPreferences() override;
    
    typedef struct feedback_s {
        bool zEn, yawEn, rollEn, pitchEn, rotateEn;
        float kpyaw, kdyaw, kiyaw, kiyawrate, yawRateIntRange;
        float kpz, kdz, kiz, z_int_low, z_int_high;
        float kproll, kdroll;
        float servoBeta, servoRange, botZlim, pitchOffset, pitchInvert;
        float lx;
    } feedback_t;
    
    // Sensor interface
    SensorSuite sensorsuite; 

    feedback_t PDterms;

    // List of the variables that need persistant storage
    float z_integral = 0;
    float yaw_integral = 0;
    float yawrate_integral = 0;
    float groundZ = 0; 

private:

    void addFeedback(float sensors[MAX_SENSORS], float controls[], float feedbackControls[]);
    void getOutputs(float sensors[MAX_SENSORS], float controls[], float outputs[]);

    float clamp(float val, float minVal, float maxVal);
    float adjustAngle(float angle);

    // Contains all the ground station constants

    
};


#endif //BLIMPSWARM_FULLBICOPTER_H
