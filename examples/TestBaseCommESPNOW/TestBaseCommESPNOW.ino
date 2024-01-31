

#include <Arduino.h>
#include "BlimpSwarm.h"
#include "comm/LLC_ESPNow.h"
#include "comm/BaseCommunicator.h"
#include "util/data_types.h"



//ESPNowComm espNowComm;
LowLevelComm* espNowComm = new LLC_ESPNow();
BaseCommunicator* baseComm = new BaseCommunicator(espNowComm);

int NUM_CONTROL_PARAMS = 13;
uint8_t mac_addr[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};

uint8_t receivedMsg[MAX_DATA_SIZE]; int receivedDataLen;

void setup() {
    // Start the serial communication
    Serial.begin(115200);

    Serial.println("Start ESPNOW");
    


    espNowComm->init();  //fixme this should be in the communicator
    //LowLevelComm* espNowComm = new ESPNowComm();


    baseComm->setMainBaseStation(mac_addr);

}

void loop() {




    ReceivedData* responseData = new ReceivedData();
    responseData->values[0] = 1;
    responseData->values[1] = 2;
    responseData->values[2] = 4;
    responseData->values[3] = 8;



    // Send a package
    espNowComm->sendData(mac_addr, (uint8_t *) responseData, sizeof(ReceivedData));
    // Same command but using the communicator
    baseComm->sendMeasurements(responseData);


    // Test basecomm class
    if (baseComm->readNewMessages()) {

        if (baseComm->isNewMsgCmd()){
          ControlInput cmdMsg = baseComm->receiveMsgCmd();
          Serial.print("Cmd arrived ");
          
          Serial.print(":");

          int n = sizeof(cmdMsg.params)/ sizeof(cmdMsg.params[0]);
          for(int i = 0; i < n; ++i) {
              Serial.print(cmdMsg.params[i]); // Print each byte in hexadecimal
              Serial.print(" "); // Print a space between bytes for readability
          }
          Serial.println();
        }
    }


    // New long data arrived?
    if (espNowComm->newData()){



        espNowComm->receiveData(receivedMsg, receivedDataLen);

        //receivedData.insert(receivedData.end(), data, data + length);

        Serial.print("Data arrived ");
        Serial.print(receivedDataLen);
        Serial.print(": ");
        for(int i = 0; i < receivedDataLen; ++i) {
            Serial.print(receivedMsg[i], HEX); // Print each byte in hexadecimal
            Serial.print(" "); // Print a space between bytes for readability
        }
        Serial.println();
    }



  sleep(1);
}
