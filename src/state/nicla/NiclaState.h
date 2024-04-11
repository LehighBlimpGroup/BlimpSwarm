
#include "state/RobotStateMachine.h"
#include "state/nicla/NiclaConfig.h"


// the basic fundamental class of Nicla State
class NiclaState : public RobotState {
protected:

    hist_t* hist;
    nicla_t terms; 

    bool detected(float sensors[]) ;
    bool closeToGoal(float sensors[]) ;
    
    virtual RobotState* statetransitions(float sensors[], float controls[]) = 0;
    virtual void behavior(float sensors[], float controls[], float outControls[]) = 0;
public:
    NiclaState();

    // function to update state based on sensors and controls
    RobotState* update(float sensors[], float controls[], float outControls[]) ;
};


// activates a random walk algorith which excels in uncertain environments
class LevyWalk : public NiclaState {
protected:
    unsigned long SpiralTimer = 0;
    unsigned long levyDuration = 0;
    float levyYaw = 0;
    float yawCurr = 0;
    unsigned long levyTimer = 0;
    
    RobotState* statetransitions(float sensors[], float controls[]) ;

    // levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
    void behavior(float sensors[], float controls[], float outControls[]) ;


public:
    LevyWalk();
};

// activates a spiral algorithm which systematically searches
class Spiral : public NiclaState {
protected:
    unsigned long spiralDuration = 0;
    float SpiralYaw = 0;
    unsigned long SpiralTimer = 0;
    
    RobotState* statetransitions(float sensors[], float controls[]) ;

    // levy walk is a random levy walk algorithm which is good for 'hunting' in a random environment
    void behavior(float sensors[], float controls[], float outControls[]) ;


public:
    Spiral();
};


// actively moves towards a detected goal
class MoveToGoal : public NiclaState {
protected:
    
    RobotState* statetransitions(float sensors[], float controls[]) ;

    // moves towards the observed goal in the environment
    void behavior(float sensors[], float controls[], float outControls[]) ;


public:
    MoveToGoal();
};


// Uses manual commands from ground station
class ManualState : public NiclaState {
protected:
    
    RobotState* statetransitions(float sensors[], float controls[]) ;

    // moves towards the observed goal in the environment
    void behavior(float sensors[], float controls[], float outControls[]) ;


public:
    ManualState();
};


// passively charges towards goal location
class ChargeGoal : public NiclaState {
protected:
    unsigned long charge_timer = 0;
    RobotState* statetransitions(float sensors[], float controls[]) ;

    // moves towards the observed goal in the environment
    void behavior(float sensors[], float controls[], float outControls[]) ;


public:
    ChargeGoal();
};
