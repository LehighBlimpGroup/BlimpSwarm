//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_ROBOT_H
#define BLIMPSWARM_ROBOT_H


#define PI 3.1416

class Robot {
private:
    // Pure virtual function for variable settings

public:
    // Assume a fixed maximum size for the arrays
    static const int MAX_SENSORS = 21;

    // Pure virtual function for sensing
    // Returns the number of elements filled in the array
    virtual int sense(float sensors[MAX_SENSORS]) = 0;

    // Pure virtual function for actuate
    // Takes an array of actuator commands and its size
    virtual bool actuate(const float actuators[], int size) = 0;

    virtual void startup();

    // Takes an array of higher-level control commands and its size
    // Also takes the sensor array
    // Calls the lower-level actuate automatically 
    virtual bool control(float sensors[MAX_SENSORS], float controls[], int size ) = 0;

    virtual void calibrate();

    // reads from Preferences library to initialize variables
    virtual void getPreferences();

    virtual ~Robot() {}
};



#endif
