#include "BlimpSwarm.h"
#include "comm/ESPNowComm.h"
#include <Arduino.h>

ESPNowComm espNowComm;

void setup() {
    // Start the serial communication
    Serial.begin(115200);

    Serial.println("Start ESPNOW");
    


    espNowComm.init();
    //LowLevelComm* espNowComm = new ESPNowComm();


}

void loop() {
    //TODO: check if new data arrived
    //TODO: Print new data
}
