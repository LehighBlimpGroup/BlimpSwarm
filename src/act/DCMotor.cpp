//
// Created by dav on 2/9/24.
//

#include "DCMotor.h"



DCMotor::DCMotor(int minVal, int maxVal, int offsetVal, int pinVal): Actuator(minVal, maxVal, offsetVal, pinVal) {


    pinMode(pinVal, OUTPUT);
    PIN_DCMOTOR = pinVal; // Initialize PIN_DCMOTOR here

}



void DCMotor::act(float value){

    int power = constrain(value, 0, 255) ; // constrain on power

    analogWrite(PIN_DCMOTOR, power);
}