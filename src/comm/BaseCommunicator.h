//
// Created by dav on 1/28/24.
//

#ifndef BLIMPSWARM_BASECOMMUNICATOR_H
#define BLIMPSWARM_BASECOMMUNICATOR_H


//class BaseCommunicator {
//public:
//    void setMainBaseStation(const uint8_t mac_addr[6]);
//
//    // During boot, off, and periodically just ping the stations.
//    void addPingStation(const uint8_t mac_addr[6]);
//    void pingStations();
//
//    // Send measurements to base station
//    bool sendMeasurements(const ReceivedData measurements);
//
//    // Commands from Base station
//    ControlInput receiveCommand();
//    bool newReceivedCommand();
//
//    // Send all params to the base station
//    bool sendParams();
//
//
//private:
//    LowLevelComm comm; // Object that manager the low level communication
//    uint8_t main_station_mac[6];  // Main station mac
//
//    static const int MAX_PING_STATIONS = 5; // Maximum number of ping stations
//    uint8_t pingStations[MAX_PING_STATIONS][6]; // Array to store MAC addresses of ping stations
//    int numPingStations = 0; // Number of ping stations currently added
//
//
//
//    updateParam(const uint8_t* data, unsigned int length);
//};


#endif //BLIMPSWARM_BASECOMMUNICATOR_H