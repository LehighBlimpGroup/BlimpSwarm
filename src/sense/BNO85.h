#ifndef BNO85_h
#define BNO85_h

#include <Wire.h>
#include "SensorInterface.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

class BNO85 : public SensorInterface {
private:
    BNO08x myIMU;
    bool bnoOn = false;
    float sensorValues[6]; // Array to store roll, pitch, yaw, rollRate, pitchRate, yawRate
    // Define sensor data structure here if needed

public:
    BNO85();
    void startup() override;
    bool update() override;
    float* readValues(int& count) override;
    void setReports();
    // Additional methods as necessary
};

#endif
