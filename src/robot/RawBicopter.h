/**
 * @file RawBicopter.h
 * @author David Saldana
 * @brief Robot Class that converts high-level commands directly to actuation values
 * @version 0.1
 * @date 2024-01-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef BLIMPSWARM_RAWBICOPTER_H
#define BLIMPSWARM_RAWBICOPTER_H


#include "Robot.h"
#include "act/BLMotor.h"
#include "act/AServo.h"
#include "act/LED.h"
#include <Preferences.h>

class RawBicopter : public Robot {
    public:
        /**
         * @brief Construct a new Raw Bicopter object
         * 
         */
        RawBicopter();

        /**
         * @copydoc Robot::startup()
         * 
         */
        void startup() override;
        
        /**
         * @copydoc Robot::sense()
         * 
         * @brief RawBicopter does not include any sensors
         * 
         */
        int sense(float sensors[MAX_SENSORS]) override;
        
        /**
         * @copydoc Robot::actuate()
         * 
         */
        void actuate(const float actuators[], int size) override;

        /**
         * @copydoc Robot::control()
         * 
         * @brief Directly converts array of higher-level control commands into actuation
         * 
         */
        void control(float sensors[MAX_SENSORS], float controls[], int size ) override;

        /**
         * @copydoc Robot::calibrate
         * 
         */
        void calibrate() override;

        /**
         * @copydoc Robot::arm()
         * 
         */
        void arm() override;

        /**
         * @copydoc Robot::getPreferences()
         * 
         */
        void getPreferences() override;

    private:
        BLMotor* motor1;
        BLMotor* motor2;
        AServo* servo1;
        AServo* servo2;
        LED* led;
};


#endif //BLIMPSWARM_RAWBICOPTER_H
