#include "BlimpSwarm.h"
#include "comm/LLC_ESPNow.h"
#include <Arduino.h>

//ESPNowComm espNowComm;
LowLevelComm* espNowComm = new LLC_ESPNow();



uint8_t receivedData[MAX_DATA_SIZE]; int receivedDataLength;

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
    if (espNowComm->newData()){



        espNowComm->receiveData(receivedData, receivedDataLength);

        //receivedData.insert(receivedData.end(), data, data + length);

        Serial.print("Data arrived ");
        Serial.print(receivedDataLength);
        Serial.print(": ");
        for(int i = 0; i < receivedDataLength; ++i) {
            Serial.print(receivedData[i], HEX); // Print each byte in hexadecimal
            Serial.print(" "); // Print a space between bytes for readability
        }
        Serial.println();
    }



  sleep(1);
}
