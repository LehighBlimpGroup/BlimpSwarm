#ifndef SensorSuite_h
#define SensorSuite_h


#include "SensorInterface.h"
#include "Barometer390.h"
#include "BNO85.h"
#include "BatterySensor.h"

class SensorSuite : public SensorInterface {
private:
    Barometer barometer;
    BNO85 bnoSensor;
    WeightedBatterySensor batterySensor;
    float sensorValues[15]; // Adjust based on actual data size needed
    int valueCount; // Keep track of the total number of values stored

public:
    // SensorSuite();
    // virtual ~SensorSuite();
    virtual void startup() override;
    virtual bool update() override;
    virtual float* readValues(int& count) override;
    void getPreferences();
};

#endif
