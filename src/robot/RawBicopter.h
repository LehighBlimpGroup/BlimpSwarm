//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_RAWBICOPTER_H
#define BLIMPSWARM_RAWBICOPTER_H


#include "Robot.h"

class RawBicopter : public Robot {
public:
    int sense(float sensors[MAX_SENSORS]) override;
    bool actuate(const float actuators[], int size) override;
    //void testActuators(float actuationCmd[4]) override;
};


#endif //BLIMPSWARM_RAWBICOPTER_H
