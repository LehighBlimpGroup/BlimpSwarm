/**
 * @file ChargeGoal.cpp
 * @author Edward Jeff
 * @brief Implementation of ChargeGoal.h
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "state/nicla/ChargeGoal.h"
#include "state/nicla/ManualState.h"
#include "state/nicla/LevyWalk.h"

RobotState* ChargeGoal::statetransitions(float sensors[], float controls[]) {
    if (controls[0] < 2) {
        // If the ground station requests the robot to transition to manual
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    } else {
        // In all other states
        if (hist->nicla_flag & GOAL_MODE) {
            if (millis() - charge_timer > 15000) {
                hist->z_estimator = sensors[1];
                RobotState* levyWalk = new LevyWalk();
                return levyWalk;
            } else {
                return this;
            }
        } else if (hist->nicla_flag & BALLOON_MODE) {
            if (millis() - charge_timer > 2000) {
                hist->num_captures += 1;
                if (hist->num_captures >= terms.num_captures 
                    || millis() - hist->start_ball_time > terms.time_in_ball * 1000) {
                    // If the number of charges exceeds the maximum amount or
                    // if the time in balloon mode exceeds the desired amount
                    hist->num_captures = 0;
                    hist->nicla_desired = 1;
                    hist->start_ball_time = millis();
                }
                
                hist->z_estimator = sensors[1];
                RobotState* levyWalk = new LevyWalk();
                return levyWalk;
            } else {
                return this;
            }
        } else {
            return this;
        }
    }
}

void ChargeGoal::behavior(float sensors[], float controls[], float outControls[]) {
    int edge = detected(sensors);
    if (edge == 1) {
        // if a new detection is fed in
        float _yaw = sensors[5];
        float _height = sensors[1];
        int nicla_flag = (int)sensors[NICLA_OFFSET + 0];
        float tracking_x = (float)sensors[NICLA_OFFSET + 5];
        float tracking_y = (float)sensors[NICLA_OFFSET + 2];
        float detection_y = (float)sensors[NICLA_OFFSET + 6];
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixles into a value between [0,1]

        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
    }
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_charge; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0; //tx
    outControls[4] = hist->robot_to_goal; //tz
}


ChargeGoal::ChargeGoal() : NiclaState() {
    charge_timer = millis();
}


