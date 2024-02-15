//
// Created by dav on 1/28/24.
//

#include "BaseCommunicator.h"

//
uint8_t receivedData[MAX_DATA_SIZE];
int receivedDataLength;

BaseCommunicator::BaseCommunicator(LowLevelComm* comm) : comm(comm) {
    comm->init();
}

void BaseCommunicator::setMainBaseStation(const uint8_t mac_addr[6]){
    memcpy(this->main_station_mac, mac_addr, 6); // Copy the MAC address
    this->addPingStation(mac_addr);  // The main station is also a ping station.
}


void BaseCommunicator::addPingStation(const uint8_t mac_addr[6]) {
    if (numPingStations < MAX_PING_STATIONS) {
        memcpy(pStations[numPingStations], mac_addr, 6); // Add new ping station MAC address
        numPingStations++; // Increment the count of ping stations
    } else {
        //TODO: Handle the case where the ping stations array is full
        // This could be logging an error message or ignoring the add request
    }
}

void BaseCommunicator::pingStations() {
    for (int i = 0; i < numPingStations; ++i) {
        // Construct ping message
        // Use comm.sendData(pingStations[i], pingMessage, pingMessageLength);
        //TODO: define a short message for pinging.
        // TODO: send ping
    }

}


bool BaseCommunicator::sendMeasurements(ReceivedData* measurements){
    comm->sendData(this->main_station_mac,(uint8_t *) measurements, sizeof(ControlInput));
    return false;  //fixme ??
}



bool BaseCommunicator::readNewMessages(){
    // Check for new messages
    if (comm->newData()){
        // Check if the message is a command
        comm->receiveData(receivedData, receivedDataLength);

        //TODO different types of messages
        
        if (receivedDataLength == sizeof(ControlInput)){
            msgCmd = (ControlInput *) receivedData;
            newMsgCmd = true;
        }else if(receivedDataLength== sizeof(ControlInput)){
            //TODO this type of message
        }


        return true;
    }
    return false;

}


ControlInput BaseCommunicator::receiveMsgCmd(){
    newMsgCmd = false;
    return *msgCmd;
}

bool BaseCommunicator::isNewMsgCmd(){
    readNewMessages();
    return this->newMsgCmd;
}