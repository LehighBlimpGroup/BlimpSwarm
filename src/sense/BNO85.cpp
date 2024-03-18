#include "BNO85.h"

BNO85::BNO85() {
    // Constructor logic if necessary
}

void BNO85::startup() {
    // if (bnoOn) return;
    bnoOn = false;
    startTime = micros();
    Serial.println("BNO Initialization!");
    Wire.begin(D4, D5); 
    delay(100);
    // Wire.setClock(400000);
    int tempcount = 0;
    bool connect = false;
    if (myIMU.begin(0x4A, Wire)) {
        tempcount++;
        Serial.print("  connecting! ");
        connect = myIMU.isConnected();
        Serial.println(connect);
        
        Serial.print("  ");
        
        delay(50 + tempcount * 50);
        
    } else  {
        Serial.println("Ooops, no BNO085 detected ... Check your wiring or I2C ADDR!");
        startTime = micros();
        return;
    }
    Wire.setClock(400000); //Increase I2C data rate to 400kHz
    Serial.print("  BNO started: ");
    connect = myIMU.isConnected();
    Serial.println(connect);
    myIMU.softReset();
    myIMU.modeOn();
    bnoOn = true;
    startTime = micros();
    for (int i = 0; i < 6; i++) {
        sensorValues[i] = 0.0f;
    }
    // if (myIMU.wasReset()) {
    //     setReports();
    // }
}

// Here is where you define the sensor outputs you want to receive
void BNO85::setReports() {
    while(myIMU.wasReset()){
        delay(1);
    }
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
    if (micros() - startTime > 5000000) { // every 3 seconds try to reconnect.
        BNO85::startup();
    }
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
    bool attitude = false;
    bool rate = false;
    // Serial.print("Event");
    while (myIMU.getSensorEvent()) {
        bevent = true;
        startTime = micros();
        // Update roll, pitch, yaw based on rotation vector
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR ) {
            // Serial.print("A");
            attitude = true;
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
        else if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED ) {
            // Serial.print("G");
            rate = true;
            sensorValues[3] = myIMU.getGyroX();
            sensorValues[4] = myIMU.getGyroY();
            sensorValues[5] = myIMU.getGyroZ();
        }
        if (attitude && rate) {
            break;
        }
        
    } 
    // Serial.println();
    

    // This example assumes you have a way to return or use sensorValues
    return bevent; // Indicate successful update
}

float* BNO85::readValues(int& count) {
    count = 6; // Indicate we are returning 6 values
    return sensorValues; // Return the pointer to the sensor values
}
