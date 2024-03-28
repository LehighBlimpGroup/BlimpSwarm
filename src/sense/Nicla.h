//
// Created by eddie on 3/12/24.
//

#ifndef BLIMPSWARM_NICLA_H
#define BLIMPSWARM_NICLA_H

#include "SensorInterface.h"
#include <cstddef>
#include <IBusBM.h>
#include <Arduino.h>

class Nicla : public SensorInterface {
private:
    float value[10]; // Array to store the battery voltage
    uint8_t mode = 0x81;
    unsigned long sendTime;
    

public:
    IBusBM IBus;
    Nicla();
    void startup() override;
    void startup(uint8_t setMode);
    bool update() override;
    bool update(uint8_t setMode);
    float* readValues(int& count) override;
    
    void getPreferences();
};


#endif