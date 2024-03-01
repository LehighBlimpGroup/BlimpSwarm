

// Barometer.cpp
#include "Barometer390.h"

// Barometer::Barometer() {
//     // Initialization code, similar to what you have in initBarometer()
// }

void Barometer::startup() {
    baroInitialized = false;
    int retryCount = 0;
    const int maxRetries = 5;
    const int initialDelay = 50; // Initial delay in milliseconds
    Serial.println("Initializing Barometer!");
    // Attempt to initialize the barometer
    while (!baroInitialized && retryCount < maxRetries) {
        if (bme.begin_I2C()) { // Successfully initialized
            Serial.println("Barometer Connected!");
            baroInitialized = true;
            break;
        } else { // Failed to initialize, increase retry count and delay
            retryCount++;
            Serial.print(F("Attempt "));
            Serial.print(retryCount);
            Serial.println(F(" failed, retrying..."));
            delay(initialDelay * retryCount); // Increase delay with each retry
        }
    } 

    if (!baroInitialized) {
        Serial.println(F("Could not find a valid BMP390 sensor, check wiring or try a different address!"));
        return; // Exit if unable to initialize
    }

    // Sensor initialized, configure settings
    bme.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bme.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bme.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bme.setOutputDataRate(BMP3_ODR_100_HZ);

    float groundLevel = 0.0;
    int calibrationReadings = 10;
    int discardReadings = 5; // Number of initial readings to discard
    for (int i = 0; i < discardReadings + calibrationReadings; i++) {
        if (!bme.performReading()) {
            Serial.println(F("Failed to perform reading for calibration."));
            return;
        }
        if (i >= discardReadings) { // Start accumulating after discarding initial readings
            groundLevel += bme.readAltitude(1013.25);
        }
        delay(100); // Short delay between readings
    }
    groundLevel /= calibrationReadings; // Average of the good readings
    temperature = bme.temperature;
    pressure = bme.pressure;
    Serial.print(F("Ground level calibrated to: "));
    Serial.println(groundLevel);
}



bool Barometer::update() {
    if (!bme.performReading() ) {
        return false;
    }
    temperature = bme.temperature;
    pressure = bme.pressure;

    // Calculate altitude
    // The formula for altitude based on pressure varies depending on your needs.
    altitude = bme.readAltitude(1013.25);
    
    return true; // Data was successfully updated
}

// returns [pressure, temperature, altitude]
float* Barometer::readValues(int& count) {
    static float values[3]; // Static to ensure it persists after the method returns
    values[0] = pressure;
    values[1] = temperature;
    values[2] = altitude;
    count = 3; // Indicate that we're returning 3 values
    return values;
}