/**
 * @file LevyWalk.h
 * @author Edward Jeff
 * @brief Class that represents a random walk algorithm which excels in uncertain environments
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "state/nicla/NiclaState.h"

class LevyWalk : public NiclaState {
    public:
        /**
         * @brief Construct a new Levy Walk object
         * 
         */
        LevyWalk();
    protected:
        unsigned long SpiralTimer = 0;
        unsigned long levyDuration = 0;
        float levyYaw = 0;
        float yawCurr = 0;
        unsigned long levyTimer = 0;
        float yawProgress; // Variable to track the progressive change in yaw
        bool actionUnderway = false;

        unsigned long lastSpinTime; // Timer for spinning
        unsigned long forwardDuration = 0; // Duration for a full 360 spin in milliseconds
        // bool isSpinning = true; // Flag to indicate spinning state

        unsigned angleChangeCount;
        int currentDirection;
        float angleProgress, yawRate = 0.;
        float currentYaw = 0;  // Current yaw in action
        
        /**
         * @copydoc NiclaState::statetransitions()
         * 
         * @brief Can transition to Manual or MoveToGoal states. Transitions to MoveToGoal when 
         * the target is detected.
         */
        RobotState* statetransitions(float sensors[], float controls[]);

        /**
         * @copydoc NiclaState::behavior()
         * 
         * @brief Implements an algorithm in which the height and yaw randomly fluctuates.
         * Efficient in uncertain environments where variability is high.
         */
        void behavior(float sensors[], float controls[], float outControls[]);

    private:
        bool wallDetected = false;
        float turnStartYaw = 0;
        float forward_force = 0;
};