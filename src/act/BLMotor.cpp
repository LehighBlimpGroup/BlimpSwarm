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

    int pwm = constrain(value, 0, 1);
    this->thrust.writeMicroseconds((int)(pwm * (max_thrust - min_thrust) + min_thrust));
}



void BLMotor::calibrate(){

    delay(1000);
    Serial.println("Calibrating ESCs....");
    // ESC arming sequence for BLHeli S
    thrust.writeMicroseconds(2000);
    delay(15000);

    // Back to minimum value
    thrust.writeMicroseconds(1000);
    delay(10);
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
        delay(3);
    }
    // Sweep down
    for (int i = 1050; i > 1100; i--)
    {
        thrust.writeMicroseconds(i);
        delay(3);
    }
    // Back to minimum value
    thrust.writeMicroseconds(1000);
    delay(100);
}