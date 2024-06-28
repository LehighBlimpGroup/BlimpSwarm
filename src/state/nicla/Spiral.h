#include "state/nicla/NiclaState.h"

// activates a spiral algorithm which systematically searches
class Spiral : public NiclaState {
    public:
        /**
         * @brief Construct a new Spiral object
         * 
         */
        Spiral();
    protected:
        unsigned long spiralDuration = 0;
        float SpiralYaw = 0;
        unsigned long SpiralTimer = 0;
        
        /**
         * @copydoc NiclaState::statetransitions()
         * 
         * @brief Can transition of Manual or MoveToGoal state. Transitions to MoveToGoal
         * desired target is detected.
         */
        RobotState* statetransitions(float sensors[], float controls[]);

        /**
         * @copydoc NiclaState::behavior()
         * 
         * @brief Performs a spiral that grows in size over time. Useful for testing detection
         * in an enclosed space.
         */
        void behavior(float sensors[], float controls[], float outControls[]);
};