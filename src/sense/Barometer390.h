


// Barometer.h
#ifndef Barometer_h
#define Barometer_h

#include "SensorInterface.h"
#include <Adafruit_BMP3XX.h>

class Barometer : public SensorInterface {
private:
    float altitude; // Stores the calculated altitude
    float pressure; // Stores the latest pressure reading
    float temperature; // Stores the latest temperature reading
    Adafruit_BMP3XX bme;
    bool baroInitialized = false;
    unsigned long startTime;
    float velocityZ;
    float groundLevel = 0.0;
    // Other private members as necessary...

public:
    Barometer();
    void startup() override;
    bool update() override;
    float* readValues(int& count) override;
    void getPreferences();
    // Additional Barometer-specific methods...
};


#endif