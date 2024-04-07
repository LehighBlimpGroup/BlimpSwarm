//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_SBLIMP_H
#define BLIMPSWARM_SBLIMP_H


#include "Robot.h"
#include <Arduino.h>
#include "act/BLMotor.h"
#include "act/LED.h"
#include <Preferences.h>

class SBlimp : public Robot {
public:
    SBlimp();
    void startup() override;
    int sense(float sensors[MAX_SENSORS]) override;
    bool actuate(const float actuators[], int size) override;
    bool control(float sensors[MAX_SENSORS], float controls[], int size ) override;
    void calibrate() override;
    void getPreferences() override;
    //void testActuators(float actuationCmd[4]) override;

private:

    BLMotor* motor1;
    BLMotor* motor2;
    BLMotor* motor3;
    BLMotor* motor4;
    LED* led;
};


#endif //BLIMPSWARM_SBLIMP_H
