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

    uint8_t mac_addr[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};



    ReceivedData* responseData = new ReceivedData();
    responseData->values[0] = 1;
    responseData->values[1] = 2;
    responseData->values[2] = 4;
    responseData->values[3] = 8;

    espNowComm->sendData(mac_addr, (uint8_t *) responseData, sizeof(ReceivedData));


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

  sleep(1);
}
