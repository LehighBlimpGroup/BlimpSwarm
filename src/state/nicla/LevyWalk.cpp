


#include "state/nicla/NiclaState.h"
//#include "cmath.h"

RobotState* LevyWalk::statetransitions(float sensors[], float controls[]) {

    if (controls[0] != 2){
        hist->z_estimator = sensors[1];
        RobotState* manualState = new ManualState();
        return manualState;
    }
    else if (detected(sensors)) {
        float _yaw = sensors[5];  
        float _height = sensors[1];  
        int niclaOffset = 11;
        float tracking_x = (float)sensors[niclaOffset + 1];
        float detection_y = (float)sensors[niclaOffset + 6];
        float x_cal = tracking_x / terms.n_max_x;
        hist->des_yaw = ((x_cal - 0.5)) * terms.x_strength;
        hist->robot_to_goal = _yaw + hist->des_yaw;
        float y_cal = detection_y / terms.n_max_y;
        if ( abs(x_cal - 0.5) < .16) { // makes sure yaw is in center before making height adjustments
            // z_offset += 20*((y_cal - 0.5))* subdt / sideLength;//(.75 - max(detection_h, detection_w)/max_y);
            hist->z_estimator =  ( _height + terms.y_strength * (y_cal - terms.y_thresh)) ; // integral must be on
        }
        RobotState* moveToGoal = new MoveToGoal();
        return moveToGoal;
    }
    else {
        return this; //pointer to itself
    }
}

// levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
void LevyWalk::behavior(float sensors[], float controls[], float outControls[]) {
    if (isSpinning) {
        if (millis() - spinTimer < spinDuration) {
            angleChangeCount++;  // Increment the count of angle changes
            float randomAngle = random(10, 45) / 180.0f * 3.14159;  // Randomly vary the yaw increment
            yawCurr = sensors[5] + 0.2;  // Adjust yaw in the chosen direction
            // Set control outputs
            outControls[0] = controls[0]; //ready
            outControls[1] = terms.fx_levy; //fx
            outControls[2] = hist->z_estimator; //fz
            outControls[3] = 0; //tx
            outControls[4] = yawCurr; //tz
        } else {
            // Spin complete, reset for Levy walk
            isSpinning = false;
            levyTimer = millis();
            spinTimer = millis();
            // Reset the angle change count
            angleChangeCount = 0;
            // Set new yaw target for Levy walk, incorporating random direction
            int direction = random(0, 2) * 2 - 1;  // Random direction for the new yaw target
            levyYaw = sensors[5] + direction * (random(-180, 181) / 180.0 * 3.14159);
        }
    } else {


        isSpinning = true; // Reactivate spinning after Levy walk
        spinTimer = millis();

         if (millis() - levyTimer > levyDuration) {// checks if duration for current yaw is over
            float _yaw = sensors[5];
            hist->z_estimator = sensors[1] + random(-4000, 10001) / 10000.0;
            levyTimer = millis();
            float lambda = 1.0 / 5000.0; // Adjust lambda for scaling; 5000 is the mean of the distribution
            float randomValue = random(1, 10001) / 10000.0; // Generate a random float between 0.0001 and 1
            unsigned long duration = (unsigned long)(-log(randomValue) / lambda);
            // Ensure the duration is within the desired range (0 to 10,000 ms)
            duration = duration % 30001; // Modulo to restrict within range if necessary
            levyDuration = duration;
            levyYaw = _yaw + random(-180, 180)/180.0f * 3.14;
        }

        int dt = SpiralTimer - millis();

        SpiralTimer += dt;

        if (abs(yawCurr - levyYaw) >0){
            yawCurr += (float) (1000.0f/(float)dt) * constrain(levyYaw - yawCurr, -.5, .5);
        }

        outControls[0] = controls[0]; //ready
        outControls[1] = terms.fx_levy; //fx
        outControls[2] = hist->z_estimator; //fz
        outControls[3] = 0; //tx
        outControls[4] = levyYaw; //tz

    }
}

LevyWalk::LevyWalk() : NiclaState() {
    SpiralTimer = millis();
    levyDuration = 2000;
    levyYaw = hist->robot_to_goal;
    yawCurr = hist->robot_to_goal;
    levyTimer = millis();
    isSpinning = true;
    spinTimer = millis();
//    spinDuration = ;
    angleChangeCount = 0;
    currentDirection = 1; // Initial direction for spinning
}


