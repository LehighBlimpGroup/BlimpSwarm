#ifndef BNO85_h
#define BNO85_h

#include <Wire.h>
#include "SensorInterface.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <math.h>

class BNO85 : public SensorInterface {
private:
    BNO08x myIMU;
    float pitchgamma = 0;
    float rollgamma = 0;
    float yawgamma = 0;
    bool bnoOn = false;
    float sensorValues[6]; // Array to store roll, pitch, yaw, rollRate, pitchRate, yawRate
    unsigned long restartLength = 500000;
    
    // Define sensor data structure here if needed

public:
    BNO85();
    void startup() override;
    bool update() override;
    float* readValues(int& count) override;
    void setReports();
    void getPreferences();
    unsigned long startTime;
    // Additional methods as necessary
};

#endif
