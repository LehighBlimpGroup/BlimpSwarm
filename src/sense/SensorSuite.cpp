#include "SensorSuite.h"

// SensorSuite::SensorSuite() {}

// SensorSuite::~SensorSuite() {
//     // Clean-up code if needed, e.g., delete sensor instances if owned
// }

void SensorSuite::startup() {
    // bnoSensor = new BNO85();
    // barometer = new Barometer();
    // batterySensor = new WeightedBatterySensor();
    bnoSensor.startup();
    barometer.startup();
    batterySensor.startup();
}


bool SensorSuite::update() {
    bool updated = false;
    int tempCount = 0, offset = 0;

    // Update and read values from barometer, if updated
    if (barometer.update()) {
        float* baroValues = barometer.readValues(tempCount);
        for(int i = 0; i < tempCount; ++i) {
            sensorValues[offset + i] = baroValues[i];
        }
        offset += tempCount;
        updated = true;
    }
    offset = 3;
    // Repeat the pattern for bnoSensor and batterySensor
    if (bnoSensor.update()) {
        float* bnoValues = bnoSensor.readValues(tempCount);
        for(int i = 0; i < tempCount; ++i) {
            sensorValues[offset + i] = bnoValues[i];
        }
        offset += tempCount;
        updated = true;
    }
    offset = 9;
    if (batterySensor.update()) {
        float* batteryValues = batterySensor.readValues(tempCount);
        for(int i = 0; i < tempCount; ++i) {
            sensorValues[offset + i] = batteryValues[i];
        }
        offset += tempCount;
        updated = true;
    }

    valueCount = offset; // Update the total number of valid sensor values stored
    return updated;
}

float* SensorSuite::readValues(int& count) {
    count = valueCount; // Return the count of valid sensor values
    return sensorValues; // Return the array of sensor values
}
