//
// Created by dav on 1/20/24.
//


#include "RawBicopter.h"
#include <Arduino.h>

int RawBicopter::sense(float sensors[MAX_SENSORS]) {
    // Implementation for sensing - fill the sensors array
    // Return the number of sensors used
    return 0; // Placeholder return value
}

bool RawBicopter::actuate(const float actuators[], int size) {
    // Implementation for actuation - process the actuators array
    // Return true if successful, false otherwise
    return true; // Placeholder return value
}

//void RawBicopter::testActuators(float actuationCmd[4]) {
//    int servo_delta = 1;
//    int motor_delta = 10;
//
//    if (actuationCmd[0] < 180) {
//        actuationCmd[0] += servo_delta;
//    } else if (actuationCmd[1] < 180) {
//        actuationCmd[1] += servo_delta;
//    } else if (actuationCmd[2] < 2000) {
//        actuationCmd[2] += motor_delta;
//    } else if (actuationCmd[3] < 2000) {
//        actuationCmd[3] += motor_delta;
//
//}