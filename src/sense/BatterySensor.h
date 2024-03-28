//
// Created by eddie on 2/28/24.
//

#ifndef BLIMPSWARM_BATTERYSENSOR_H
#define BLIMPSWARM_BATTERYSENSOR_H

#include "SensorInterface.h"
#include <Arduino.h>

class BatterySensor : public SensorInterface {
private:
    float value[1]; // Array to store the battery voltage
    int pin; // Analog pin where the battery is connected
    float referenceVoltage; // Reference voltage for analogRead, typically 5V or 3.3V

public:
    BatterySensor(int pin, float referenceVoltage = 5.0);
    void startup() override;
    bool update() override;
    float* readValues(int& count) override;
    void getPreferences();
};


class WeightedBatterySensor : public SensorInterface {
private:
    BatterySensor* sensor; // Assumes ownership of a BatterySensor
    float filteredValue; // Stores the exponentially weighted average
    float gamma; // Weighting factor for the filter
    int valueCount; // To comply with the readValues method signature
    
    

public:
    // Larger gamma (between 0,1) means more preference to old values
    WeightedBatterySensor( float gamma = 0.975);
    ~WeightedBatterySensor();
    void startup(int pin, float referenceVoltage);
    void startup() override;
    bool update() override;
    float* readValues(int& count) override;
    void getPreferences();
};




#endif //BLIMPSWARM_BATTERYSENSOR_H
