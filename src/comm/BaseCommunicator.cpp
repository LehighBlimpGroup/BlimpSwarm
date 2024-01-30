//
// Created by dav on 1/28/24.
//

//#include "BaseCommunicator.h"
//
//
//
//void BaseCommunicator::setMainBaseStation(const uint8_t mac_addr[6]){
//    memcpy(this->main_station_mac, mac_addr, 6); // Copy the MAC address
//    this->addPingStation(mac_addr);  // The main station is also a ping station.
//}
//
//
//void BaseCommunicator::addPingStation(const uint8_t mac_addr[6]) {
//    if (numPingStations < MAX_PING_STATIONS) {
//        memcpy(pingStations[numPingStations], mac_addr, 6); // Add new ping station MAC address
//        numPingStations++; // Increment the count of ping stations
//    } else {
//        //TODO: Handle the case where the ping stations array is full
//        // This could be logging an error message or ignoring the add request
//    }
//}
//
//void BaseCommunicator::pingStations() {}(const uint8_t mac_addr[6]) {
//
//    //TODO: define a short message for pinging.
//}
//
//
//bool BaseCommunicator::sendMeasurements(const ReceivedData measurements){
//
//
//}