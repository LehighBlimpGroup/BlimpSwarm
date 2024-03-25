
#include "RobotState.h"

class RobotStateMachine {
private:
    RobotState* currentState;

public:
    RobotStateMachine(RobotState* initState) : currentState(initState) {}

    ~RobotStateMachine() {
        delete currentState;
    }

    void update(float sensors[], float controls[], float outControls[]) {
        RobotState* nextState = currentState->update(sensors, controls, outControls);
        if (nextState != currentState) {
            delete currentState;
            currentState = nextState;
        }
    }
};
