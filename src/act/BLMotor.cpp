//
// Created by dav on 2/9/24.
//

#include "BLMotor.h"
#include <Arduino.h>



BLMotor::BLMotor(int minVal, int maxVal, int offsetVal, int pinVal, int periodHertz)
        : Actuator(minVal, maxVal, offsetVal, pinVal) {
    // Additional initialization specific to SpecificActuator

    pinMode(pinVal, OUTPUT);

    this->thrust.attach(pinVal, 1000, 2000);  //TODO 1000 and 2000 should be parameters
    this->thrust.setPeriodHertz(periodHertz);
}


void BLMotor::act(float value){
    //TODO the input should be force. We should use the calibration parameters
    //TODO the thrust is related to angular velocity
    if (value <0){
        value = 0;
    }
    value = sqrt(value);
    float pwm = constrain(value, 0, 1);
    this->thrust.writeMicroseconds((int)(pwm * (max_thrust - min_thrust) + min_thrust));
}



void BLMotor::calibrate(){

    delay(1000);
    Serial.println("Calibrating ESCs....");
    // ESC arming sequence for BLHeli S
    thrust.writeMicroseconds(2000);
    delay(8000);

    // Back to minimum value
    thrust.writeMicroseconds(1100);
    delay(8000);
    thrust.writeMicroseconds(0);
    delay(1000);
    Serial.println("Calibration completed");
}


void BLMotor::arm(){
// ESC arming sequence for BLHeli S
    thrust.writeMicroseconds(1000);
    delay(10);

    // Sweep up
    for (int i = 1050; i < 1500; i++)
    {
        thrust.writeMicroseconds(i);
        delay(6);
    }
    // Sweep down
    // for (int i = 1500; i > 1050; i--)
    // {
    //     thrust.writeMicroseconds(i);
    //     delay(6);
    // }
    // Back to minimum value
    thrust.writeMicroseconds(1000);
    delay(100);
}
