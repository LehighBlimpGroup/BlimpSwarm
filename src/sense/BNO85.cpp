#include "BNO85.h"

BNO85::BNO85() {
    // Constructor logic if necessary
}

void BNO85::startup() {
    if (bnoOn) return;
    bnoOn = false;
    Serial.println("BNO Initialization!");
    Wire.begin(D4, D5); 
    // Wire.setClock(400000);
    int tempcount = 0;
    while (!myIMU.begin(0x4A, Wire)) {
        tempcount++;
        Serial.println("  Retrying!");
        
        delay(50 + tempcount * 50);
        if (tempcount > 5) {
            Serial.println("Ooops, no BNO085 detected ... Check your wiring or I2C ADDR!");
            return;
        }
    }
    // myIMU.softReset();
    Serial.println("BNO started!");
    bnoOn = true;
    for (int i = 0; i < 6; i++) {
        sensorValues[i] = 0.0f;
    }
    // if (myIMU.wasReset()) {
    //     setReports();
    // }
}

// Here is where you define the sensor outputs you want to receive
void BNO85::setReports() {
    Serial.println("  Setting desired reports");
    if (myIMU.enableRotationVector() == true) {
        Serial.println(F("    Rotation vector enabled"));
        Serial.println(F("      Output in form roll, pitch, yaw in radians"));
    } else {
        Serial.println("    Could not enable rotation vector");
    }
    if (myIMU.enableGyro() == true) {
        Serial.println(F("    Gyro enabled"));
        Serial.println(F("      Output in form x, y, z, in radians per second"));
    } else {
        Serial.println("    Could not enable gyro");
    }
}

bool BNO85::update() {

    if (!bnoOn) {
        // Initialize sensorValues to 0 if the sensor is not on
        for (int i = 0; i < 6; i++) {
            sensorValues[i] = 0.0f;
        }
        return false;
    }

    if (myIMU.wasReset()) {
        Serial.println("Sensor was reset");
        setReports();
    }
    bool bevent = false;
    while (myIMU.getSensorEvent()) {
        bevent = true;
        // Update roll, pitch, yaw based on rotation vector
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            sensorValues[0] = myIMU.getRoll();
            sensorValues[1] = myIMU.getPitch();
            sensorValues[2] = myIMU.getYaw();
            // Adjust the angles to be within -PI to PI
            for (int i = 0; i < 3; i++) {
                while (sensorValues[i] > PI) sensorValues[i] -= 2 * PI;
                while (sensorValues[i] < -PI) sensorValues[i] += 2 * PI;
            }
        }
        // Update gyroscope rates
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
            sensorValues[3] = myIMU.getGyroX();
            sensorValues[4] = myIMU.getGyroY();
            sensorValues[5] = myIMU.getGyroZ();
        }
    } 

    // This example assumes you have a way to return or use sensorValues
    return bevent; // Indicate successful update
}

float* BNO85::readValues(int& count) {
    count = 6; // Indicate we are returning 6 values
    return sensorValues; // Return the pointer to the sensor values
}
