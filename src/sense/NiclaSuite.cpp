

#include "NiclaSuite.h"


void NiclaSuite::startup() {
    // bnoSensor = new BNO85();
    // barometer = new Barometer();
    // batterySensor = new WeightedBatterySensor();
    NiclaSuite::getPreferences();
    nicla.startup();
    sensorSuite.startup();
    for (int i = 0; i < 21; i++) {
        sensorValues[i] = 0;
    }
}


bool NiclaSuite::update() {
    bool updated = false;
    int tempCount = 0, offset = 0;

    // Update and read values from barometer, if updated
    if (sensorSuite.update()){
        // updated = true;
    }

    float* sensorSuiteValues = sensorSuite.readValues(tempCount);
    for(int i = 0; i < tempCount; ++i) {
        sensorValues[offset + i] = sensorSuiteValues[i];
    }
       
    offset += tempCount;
    // Repeat the pattern for bnoSensor and batterySensor
    
    if (nicla.update()) {
        updated = true;  
    }
    float* niclaValues = nicla.readValues(tempCount);
    for(int i = 0; i < tempCount; ++i) {
        sensorValues[offset + i] = niclaValues[i];
    }
    offset += tempCount;

    valueCount = offset; // Update the total number of valid sensor values stored
    return updated;
}

float* NiclaSuite::readValues(int& count) {
    count = valueCount; // Return the count of valid sensor values
    return sensorValues; // Return the array of sensor values
}


void NiclaSuite::getPreferences(){
    nicla.getPreferences();
    sensorSuite.getPreferences();
    return;

}