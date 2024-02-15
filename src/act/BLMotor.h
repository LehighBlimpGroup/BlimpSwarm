//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_BLMOTOR_H
#define BLIMPSWARM_BLMOTOR_H

#include "Actuator.h"
#include <ESP32Servo.h>

class BLMotor : public Actuator {
public:
    BLMotor(int minVal, int maxVal, int offsetVal, int pinVal, int periodHertz);

    void calibrate();
    void arm();

    void act(float value);

private:
    Servo thrust;
    int min_thrust = 1050;
    int max_thrust = 1750;
};


#endif //BLIMPSWARM_BLMOTOR_H
