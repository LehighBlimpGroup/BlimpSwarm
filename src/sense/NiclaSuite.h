#ifndef NiclaSuite_h
#define NiclaSuite_h


#include "SensorSuite.h"
#include "Nicla.h"


class NiclaSuite : public SensorInterface {
private:
    Nicla nicla;
    SensorSuite sensorSuite;
    float sensorValues[21]; // Adjust based on actual data size needed
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
