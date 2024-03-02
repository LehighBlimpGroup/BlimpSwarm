//
// Created by edward on 3/1/24.
//

#ifndef BLIMPSWARM_FULLBICOPTER_H
#define BLIMPSWARM_FULLBICOPTER_H



#include "FullBicopter.h"



typedef struct feedback_s {
    bool zEn, yawEn, rollEn, pitchEn, rotateEn;
    float kpyaw, kdyaw, kiyaw, kiyawrate, yawRateIntRange;
    float kpz, kdz, kiz, z_int_low, z_int_high;
    float kproll, kdroll;
    float lx;
} feedback_t;

class FullBicopter : public FullBicopter {
public:
    FullBicopter();
    
    void getPreferences() override;
    
    

private:
    void addFeedback(float sensors[MAX_SENSORS], float controls[], float feedbackControls[]) override;
    void getOutputs(float sensors[MAX_SENSORS], float controls[], float outputs[]) override;



    // Contains all the ground station constants
    feedback_t PDterms override;

    
};


#endif //BLIMPSWARM_FULLBICOPTER_H
