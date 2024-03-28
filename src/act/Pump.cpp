//
// Created by dav on 2/9/24.
//

#include "Pump.h"



Pump::Pump(int minVal, int maxVal, int offsetVal, int pinVal): Actuator(minVal, maxVal, offsetVal, pinVal) {


    pinMode(pinVal, OUTPUT);


}


void Pump::act(float value){

    int power = constrain(value, 0, 255) ; // constrain on power

    analogWrite(pinVal, power);

}