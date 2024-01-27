#include "BlimpSwarm.h"
#include "comm/ESPNowComm.h"
#include <Arduino.h>

//ESPNowComm espNowComm;
LowLevelComm* espNowComm = new ESPNowComm();

void setup() {
    // Start the serial communication
    Serial.begin(115200);

    Serial.println("Start ESPNOW");
    


    espNowComm->init();
    //LowLevelComm* espNowComm = new ESPNowComm();


}

void loop() {
    int NUM_CONTROL_PARAMS = 13;

    // New long data arrived?
    if (espNowComm->newLongData()){
        Serial.print("New long data arrived: ");

        ControlInput incomingData = espNowComm->receiveLongData();

        Serial.print("Control params: ");
        for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
        {
            Serial.print(incomingData.params[i]);
            if (i < NUM_CONTROL_PARAMS - 1)
            {
                Serial.print(", ");
            }
        }
        Serial.println();
    }


    // New short data arrived?
    if (espNowComm->newShortData()){
        Serial.print("Short data arrived: ");

        ReceivedData incomingData = espNowComm->receiveShortData();
    }

}
