// File: BlimpSwarm/src/comm/BaseCommunicator.h

#ifndef BLIMPSWARM_BASECOMMUNICATOR_H
#define BLIMPSWARM_BASECOMMUNICATOR_H

#include <cstdint>
#include <Arduino.h>
#include "LowLevelComm.h"
#include "util/DataTypes.h"
#include "util/ParamManager.h"
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>

// Define MAX_PING_STATIONS
#define MAX_PING_STATIONS 5

class BaseCommunicator {
public:
    /**
     * @brief Construct a new Base Communicator object
     * 
     * @param comm The type of communication station to create
     */
    explicit BaseCommunicator(LowLevelComm* comm); // Constructor declaration

    /**
     * @brief Destroy the Base Communicator object
     */
    ~BaseCommunicator(); // Destructor declaration

    /**
     * @brief Sets the main station parameter to the current station
     */
    void setMainBaseStation();

    /**
     * @brief Adds a ping station to the list of stations to send pings to.
     * 
     * @param mac_addr The MAC address of the ping station to be added to memory
     */
    void addPingStation(const uint8_t mac_addr[6]);

    /**
     * @brief Sends a ping message to each station in the list
     */
    void pingStations();

    /**
     * @brief Sends measurements to the base station
     * 
     * @param measurements The data to be sent to the base station
     * @return true Data was successfully sent
     * @return false Data was not sent
     */
    bool sendMeasurements(ReceivedData* measurements);

    /**
     * @brief Checks for new data and reads it if new data was received.
     * 
     * @return true Data was successfully received
     * @return false Data was not received
     */
    bool readNewMessages();
    
    /**
     * @brief Takes the data from readNewMessages() and returns it to be used.
     * 
     * @return ControlInput Returns the commands received from the base station
     */
    ControlInput receiveMsgCmd();

    /**
     * @brief Reads the latest message and checks if it is a new message.
     * 
     * @return true The message was a new message
     * @return false The message isn't a new message
     */
    bool isNewMsgCmd();

private:
    LowLevelComm* comm; // Object that manages the low level communication
    uint8_t main_station_mac[6];  // Main station MAC

    uint8_t pStations[MAX_PING_STATIONS][6]; // Array to store MAC addresses of ping stations
    int numPingStations = 0; // Number of ping stations currently added
    Preferences preferences;

    unsigned long previousMillis = 0; // Stores the last time a message was sent
    const long interval = 333; // Interval at which to run the sender (milliseconds)

    // Messages
    ControlInput* msgCmd = nullptr;
    bool newMsgCmd = false;
    ReceivedData* msgMeasure = nullptr;
    bool newMeasure = false;

    // ESP-NOW callback handling
    void handleReceive(const esp_now_recv_info_t* recvInfo, const uint8_t *incomingData, int len);
    void handleSend(const uint8_t *mac_addr, esp_now_send_status_t status);

    // Static instance pointer for callback delegation
    static BaseCommunicator* instance;

    // Static callback functions
    static void onDataReceiveCallback(const esp_now_recv_info_t* recvInfo, const uint8_t *incomingData, int len);
    static void onDataSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);
};

#endif // BLIMPSWARM_BASECOMMUNICATOR_H
