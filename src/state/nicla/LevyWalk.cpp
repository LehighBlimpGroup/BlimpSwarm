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


RobotState* LevyWalk::statetransitions(float sensors[], float controls[]) {
    if (controls[0] < 2){
        // If the ground station requests the robot to transition to manual
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    } else if (terms.state != hist->nicla_desired) {
        // If the current state is different than the desired state
        // // Serial.println("New Levywalk created");

        RobotState* levyWalk = new LevyWalk();
        return levyWalk;
    } else if (detected(sensors)) {  
        // // Serial.println("Target detected, moved to moveGoal");
        // If the target is detected
        float _yaw = sensors[5];
        int nicla_flag = (int)sensors[NICLA_OFFSET + 0];
        float tracking_x = (float)sensors[NICLA_OFFSET + 1];
        
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixels into a value between [0,1]

        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
        
        RobotState* moveToGoal = new MoveToGoal();
        // return moveToGoal;
        return this;
    } else if (sensors[11] < WALL_DISTANCE_THRESH) {
        // Serial.println("Wall mode entered");

        // Wall detected, initiate turn
        wallDetected = true;
        turnStartYaw = sensors[5]; // Store current yaw
        return this; // Stay in LevyWalk state, but with wall avoidance active
    } else if (millis() - hist->start_ball_time > terms.time_in_ball * 1000) {
        // If the robot has been in LevyWalk state for more than the designated time
        hist->num_captures = 0;
        hist->nicla_desired = 1;
        hist->start_ball_time = millis();
        return this;
    } else {
        return this; //pointer to itself
    }
}

void LevyWalk::behavior(float sensors[], float controls[], float outControls[]) {
    unsigned long currentTime = millis();
    currentYaw = sensors[5];

    if (wallDetected) {

        // Calculate the target yaw (90 degrees turn)
        Serial.println("Wall Detected");
        // Serial.println(sensors[11]);
        // Serial.println(turnStartYaw,currentYaw);
        float targetYaw = turnStartYaw + M_PI / 2.0; // Add 90 degrees (in radians)
        // float targetYaw = turnStartYaw + random( M_PI/2.0 - WallTurnRange, M_PI/2.0);
        
        // Normalize the target yaw to be between -π and π
        while (targetYaw > M_PI) targetYaw -= 2 * M_PI;
        while (targetYaw < -M_PI) targetYaw += 2 * M_PI;

        // Current yaw
        

        // Calculate the difference
        float yawDiff = targetYaw - currentYaw;
        
        // Normalize the difference to be between -π and π
        while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
        while (yawDiff < -M_PI) yawDiff += 2 * M_PI;

        // If we're close to the target yaw, stop turning
        if (abs(yawDiff) < 0.5) { // 0.1 radians is about 5.7 degrees
            wallDetected = false;
            currentYaw = targetYaw; // Reset current yaw to new direction
        } else {
            // Otherwise, keep turning
            // Serial.println("Yaw Set by wall detection");
            unsigned long dt = currentTime - SpiralTimer;
            yawRate -= 0.02* (dt / 1000.0);  // Gradually increase the yaw rate
            // angleProgress += yawRate * (dt / 1000.0);  // Update yaw based on the elapsed time in seconds
            angleProgress = 0.5;
            // currentYaw += (yawDiff > 0 ? angleProgress : -1*angleProgress); // Turn at a fixed rate
            currentYaw = targetYaw ;
        }
        
    } else {
        // Spiral state
        Serial.println("Normal Levywalk behavior");
        if (isSpinning) {
            unsigned long timeElapsed = currentTime - spinTimer;

            if (timeElapsed < spinDuration) {
            // if (true) {
                unsigned long dt = currentTime - SpiralTimer;  // Calculate the elapsed time since the last update
                if (dt > 0) {
                    yawRate -= 0.02* (dt / 1000.0);  // Gradually increase the yaw rate
                    yawRate = constrain(yawRate, 0, .5);  // Limit yaw rate to max value
                    currentYaw += yawRate * (dt / 1000.0);  // Update yaw based on the elapsed time in seconds
                    SpiralTimer = currentTime;  // Update the SpiralTimer to the current time
                    // currentYaw = angleProgress;  // Synchronize currentYaw with angleProgress
                    // currentYaw += 0;
                }
            } else {
                isSpinning = true;  // Stop spinning after completing the duration
                SpiralTimer = currentTime;  // Reset SpiralTimer for the next behavior
                spinTimer = currentTime;
                spinDuration = random(5000, 20000);
                yawRate = 0.5;
                currentYaw = sensors[5] + random(50, 120)/180.0f * 3.14;
                currentYaw = 0;
                if (hist->nicla_desired == 1){ // goal mode
                    hist->z_estimator = terms.goal_height + random(-15000, 15000) / 10000.0f;
                    
                } else { // ball mode
                    float d = random(-12000, 15000) / 10000.0;
                    hist->z_estimator = constrain(sensors[1] + d, 1, terms.goal_height-1);
                    if (sensors[1] >= terms.goal_height-1.5 || sensors[1] <= 1){
                        hist->z_estimator = (terms.goal_height-1)/2;
                    }
                }
            }
        } else {
            isSpinning = true;  // Stop spinning after completing the duration
            yawRate = 0.5;
        }
    }

    // Set control outputs for both behaviors
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0;  // Other control
    // Serial.print("Published desired yaw  : "); Serial.print(currentYaw); Serial.print("Current yaw: "); Serial.println(sensors[5]);
    outControls[4] = currentYaw;  // Set the updated yaw based on behavior
}



LevyWalk::LevyWalk() : NiclaState() {
    SpiralTimer = millis();
    levyDuration = 10000;
    levyYaw = hist->robot_to_goal;
    yawCurr = hist->robot_to_goal;
    currentYaw = hist->robot_to_goal;
    angleProgress = hist->robot_to_goal;
    levyTimer = millis();
    isSpinning = true;
    spinTimer = millis();
//    spinDuration = ;
    yawRate = 0.2;
    spinDuration = 5000;
    angleChangeCount = 0;
    currentDirection = 1; // Initial direction for spinning
}


