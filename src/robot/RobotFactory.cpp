//
// Created by dav on 1/20/24.
//


#include "RobotFactory.h"

Robot* RobotFactory::createRobot(const String& type) {
    if (type== "RawBicopter") {
        return new RawBicopter();
    } else if (type== "FullBicopter") {
        return new FullBicopter();
    }else if (type== "CustomBicopter") {
        return new CustomBicopter();
    }else if (type== "SBlimp") {
        return new SBlimp();
    }
        // Add cases for other robot types here
    else {
        return nullptr;
    }
}