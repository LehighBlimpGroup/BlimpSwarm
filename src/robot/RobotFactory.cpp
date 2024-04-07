//
// Created by dav on 1/20/24.
//


#include "RobotFactory.h"

Robot* RobotFactory::createRobot(const String& type) {
    if (type== "RawBicopter") {
        Robot* robot = new RawBicopter();
        robot->startup();
        return robot;
    } else if (type== "FullBicopter") {
        Robot* robot = new FullBicopter();
        robot->startup();
        return robot;
    }else if (type== "CustomBicopter") {
        Robot* robot = new CustomBicopter();
        robot->startup();
        return robot;
    }else if (type== "SBlimp") {
        Robot* robot = new SBlimp();
        robot->startup();
        return robot;
    }
        // Add cases for other robot types here
    else {
        return nullptr;
    }
}