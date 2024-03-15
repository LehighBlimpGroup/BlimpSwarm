//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_ROBOTFACTORY_H
#define BLIMPSWARM_ROBOTFACTORY_H


#include "Arduino.h"
#include "Robot.h"

// Include robots
#include "RawBicopter.h"
#include "FullBicopter.h"
#include "CustomBicopter.h"
#include "SBlimp.h"


class RobotFactory {
public:
    static Robot* createRobot(const String& type);
};



#endif //BLIMPSWARM_ROBOTFACTORY_H
