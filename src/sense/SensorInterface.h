//
// Created by dav on 2/10/24.
//

#ifndef BLIMPSWARM_SENSOR_H
#define BLIMPSWARM_SENSOR_H

#include <Preferences.h>

class SensorInterface {
public:
    // Virtual destructor to ensure derived class destructors are called
    virtual ~SensorInterface() {}

    // Initializes sensors in case they are unable to run on startup (Example: I2C devices)
    virtual void startup() = 0;

    // Updates the sensor values. Must be implemented by derived classes.
    virtual bool update() = 0;

    // Reads all the sensor values and returns them along with the size.
    // The return type is a pointer to a float array.
    // Implementations should also set the output parameter `count` to indicate the size of the array.
    virtual float* readValues(int& count) = 0;

    // gets ground station parameters for tuning
    virtual void getPreferences();
};


#endif //BLIMPSWARM_SENSOR_H
