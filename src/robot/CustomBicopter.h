//
// Created by edward on 3/1/24.
//

#ifndef BLIMPSWARM_CUSTOMBICOPTER_H
#define BLIMPSWARM_CUSTOMBICOPTER_H



#include "FullBicopter.h"
#include <Arduino.h>




class CustomBicopter : public FullBicopter {
public:
    CustomBicopter();
    
    bool control(float sensors[MAX_SENSORS], float controls[], int size ) override;
    void getPreferences() override;
    
    

private:
    void addFeedback(float sensors[MAX_SENSORS], float controls[], float feedbackControls[]);
    void getOutputs(float sensors[MAX_SENSORS], float controls[], float outputs[]);


    
    typedef struct custom_s {
        bool info;
    } custom_t;
    // Contains all the ground station constants
    custom_s customItems;

    
};


#endif //BLIMPSWARM_FULLBICOPTER_H
