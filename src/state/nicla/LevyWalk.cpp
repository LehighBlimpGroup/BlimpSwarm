/**
 * @file LevyWalk.cpp
 * @author Edward Jeff
 * @brief Implementation of LevyWalk.h
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "state/nicla/LevyWalk.h"
#include "state/nicla/ManualState.h"
#include "state/nicla/MoveToGoal.h"

LevyWalk::LevyWalk() : NiclaState() {
    spiralTimer = millis();

    lastSpinTime = millis(); // Timer for spinning
    exploreDuration = 2500; // Duration for a full 360 spin in milliseconds
    spiralDuration = exploreDuration * terms.percent_spiral;

    currentYaw = 0;  // Current yaw in action
    hist->start_ball_time = millis();
    currentYaw = hist->robot_to_goal;
    yawRate = terms.levy_yaw;
}


RobotState* LevyWalk::statetransitions(float sensors[], float controls[]) {
    if (controls[0] < 2){
        // If the ground station requests the robot to transition to manual
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    } else if (terms.state != hist->nicla_desired) {
        // If the current state is different than the desired state
        RobotState* levyWalk = new LevyWalk();
        return levyWalk;
    } else if (detected(sensors)) {  
        // If the target is detected
        float _yaw = sensors[5];
        int nicla_flag = (int)sensors[NICLA_OFFSET + 0];
        float tracking_x = (float)sensors[NICLA_OFFSET + 1];
        
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixels into a value between [0,1]

        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
        
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    } else if (!hist->goForward && sensors[11] < terms.wall_thresh) {
        // Wall detected, initiate turn
        wallDetected = true;
        turnStartYaw = sensors[5]; // Store current yaw
        return this; // Stay in LevyWalk state, but with wall avoidance active
    } else if (millis() - hist->start_ball_time > terms.time_in_mode * 1000) {
        // If the robot has been in LevyWalk state for more than the designated time
        hist->num_captures = 0;
        hist->nicla_desired = !hist->nicla_desired;
        hist->start_ball_time = millis();
        return this;
    } else {
        hist->robot_to_goal = sensors[5];
        return this; //pointer to itself
    }
}

void LevyWalk::behavior(float sensors[], float controls[], float outControls[]) {
    unsigned long currentTime = millis();
    if (wallDetected) {
        // Calculate the target yaw (90 degrees turn)
        float targetYaw = turnStartYaw + M_PI / 2.0; // Add 90 degrees (in radians)
        
        // Normalize the target yaw to be between -π and π
        while (targetYaw > M_PI) targetYaw -= 2 * M_PI;
        while (targetYaw < -M_PI) targetYaw += 2 * M_PI;

        // Calculate the difference
        float yawDiff = targetYaw - sensors[5];
        
        // Normalize the difference to be between -π and π
        while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
        while (yawDiff < -M_PI) yawDiff += 2 * M_PI;

        // If we're close to the target yaw, stop turning
        if (abs(yawDiff) < 0.5) { // 0.1 radians is about 5.7 degrees
            exploreDuration = random(5000, 10000);
            lastSpinTime = currentTime;
            wallDetected = false;
        } else {
            currentYaw = targetYaw;
        }
        
    } else {

        // Spiral state
        // Serial.println("Normal Levywalk behavior");
        unsigned long timeElapsed = currentTime - lastSpinTime;
        if(timeElapsed >= exploreDuration + spiralDuration) { // random yaw and height
            if(hist->goForward) {
                hist->goForward = false;
            }
            lastSpinTime = currentTime;
            yawRate = 0.5;
            exploreDuration = random(0, 5000);
            spiralDuration = random(15000, 20000); // TODO: change so that the percent spiral makes a spiral duration

            currentYaw = sensors[5] + random(30, 90)/180.0f * 3.14;
            float random_height = random(-terms.fz_levy*10000, terms.fz_levy*10000) / 10000.0;
            hist->z_estimator = sensors[1] + random_height;
        } else if(!hist->goForward) { // spiral
            if(timeElapsed < exploreDuration) {
                
            } else if(timeElapsed < spiralDuration + exploreDuration) {
                unsigned long dt = currentTime - spiralTimer;  // Calculate the elapsed time since the last update
                if (dt > 0) {
                    yawRate -= terms.levy_yaw * (dt / 1000.0);  // Gradually increase the yaw rate
                    yawRate = constrain(yawRate, 0.1, .5);  // Limit yaw rate to max value
                    currentYaw += yawRate * (dt / 1000.0);  // Synchronize currentYaw with angleProgress
                }
            }
            spiralTimer = currentTime;
        }

        if (sensors[1] >= terms.default_height + terms.height_range ||
            sensors[1] <= terms.default_height - terms.height_range) {
            hist->z_estimator = terms.default_height;
        }
    }

    // Set control outputs for both behaviors
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0;  // Other control
    outControls[4] = currentYaw;  // Set the updated yaw based on behavior
}


