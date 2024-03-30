//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_DCMOTOR_H
#define BLIMPSWARM_DCMOTOR_H

#include "Actuator.h"
#include <ESP32Servo.h>

class DCMotor : public Actuator{
public:
    DCMotor(int minVal, int maxVal, int offsetVal, int pinVal);

    void act(float value);

private:
      int PIN_DCMOTOR;
//    Servo servo;
//    int servo_min = 550;   //FIXME set it somewhere else
//    int servo_max = 2450;  //FIXME set it somewhere else
};


#endif //BLIMPSWARM_PUMP_H
