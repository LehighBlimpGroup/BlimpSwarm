


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
    static float angleProgress = sensors[5];  // Maintain state between calls, initialized to the first sensor yaw
    static float yawRate = 0.0;  // Initial yaw rate, also static to maintain state
    static float currentYaw = sensors[5];  // Current yaw in action
    static float targetYaw = sensors[5];  // Target yaw for Levy Walk
    const float maxYawRate = 0.5;  // Maximum yaw rate to avoid overly rapid rotation
    const float maxYawIncrement = 0.0001;  // Maximum incremental change in yaw per update
    const unsigned long spinDuration = 20000;  // Duration of the spin in milliseconds
    static unsigned long SpiralTimer = millis();
    static unsigned long levyTimer = millis();
    static bool isSpinning = true;

    unsigned long currentTime = millis();

    // Spiral state
    if (isSpinning) {
        unsigned long timeElapsed = currentTime - SpiralTimer;

        if (timeElapsed < spinDuration) {
            unsigned long dt = currentTime - SpiralTimer;  // Calculate the elapsed time since the last update
            if (dt > 0) {
                yawRate += 0.0001;  // Gradually increase the yaw rate
                yawRate = std::min(yawRate, maxYawRate);  // Limit yaw rate to max value
                angleProgress += yawRate * (dt / 1000.0);  // Update yaw based on the elapsed time in seconds
                SpiralTimer = currentTime;  // Update the SpiralTimer to the current time
                currentYaw = angleProgress;  // Synchronize currentYaw with angleProgress
            }
        } else {
            isSpinning = false;  // Stop spinning after completing the duration
            SpiralTimer = currentTime;  // Reset SpiralTimer for the next behavior
            levyTimer = currentTime;  // Prepare for Levy Walk
            targetYaw = sensors[5];  // Reset targetYaw for Levy Walk
        }
    }

    // Levy Walk state
    else {
        if (currentTime - levyTimer > spinDuration) {

//            float _yaw = sensors[5];
//            hist->z_estimator = sensors[1] + random(-4000, 10001) / 10000.0;
//            levyTimer = millis();
//            float lambda = 1.0 / 5000.0; // Adjust lambda for scaling; 5000 is the mean of the distribution
//            float randomValue = random(1, 10001) / 10000.0; // Generate a random float between 0.0001 and 1
//            unsigned long duration = (unsigned long)(-log(randomValue) / lambda);
//            // Ensure the duration is within the desired range (0 to 10,000 ms)
//            duration = duration % 30001; // Modulo to restrict within range if necessary
//            levyDuration = duration;
//            levyYaw = _yaw + random(-180, 180)/180.0f * 3.14;

            targetYaw = sensors[5] + static_cast<float>(random(-180, 180)) / 180.0f * 3.14159;  // Compute new target yaw in radians
            levyTimer = currentTime;  // Reset levyTimer after Levy Walk completes
            isSpinning = true;  // Prepare to start spinning after Levy Walk
            SpiralTimer = currentTime;  // Start timing the spin
        }

        // Smooth transition to new yaw
        if (fabs(currentYaw - targetYaw) > maxYawIncrement) {
            currentYaw += (currentYaw < targetYaw ? maxYawIncrement : -maxYawIncrement);
        } else {
            currentYaw = targetYaw;  // If the difference is very small, directly set to target
        }
    }

    // Set control outputs for both behaviors
    outControls[0] = controls[0]; //ready
    outControls[1] = terms.fx_levy; //fx
    outControls[2] = hist->z_estimator; //fz
    outControls[3] = 0;  // Other control
    outControls[4] = currentYaw;  // Set the updated yaw based on behavior
}






LevyWalk::LevyWalk() : NiclaState() {
    SpiralTimer = millis();
    levyDuration = 10000;
    levyYaw = hist->robot_to_goal;
    yawCurr = hist->robot_to_goal;
    levyTimer = millis();
    isSpinning = true;
    spinTimer = millis();
//    spinDuration = ;
    angleChangeCount = 0;
    currentDirection = 1; // Initial direction for spinning
}


