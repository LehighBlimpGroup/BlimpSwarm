// File: BlimpSwarm/src/comm/BaseCommunicator.cpp

/**
 * @file BaseCommunicator.cpp
 * @brief Implementation of BaseCommunicator.h
 */

#include "BaseCommunicator.h"
#include <esp_now.h>
#include <WiFi.h>

// Initialize static instance pointer
BaseCommunicator* BaseCommunicator::instance = nullptr;

// Constructor
BaseCommunicator::BaseCommunicator(LowLevelComm* comm) : comm(comm), msgCmd(nullptr), msgMeasure(nullptr) {
    // Assign the global instance pointer
    if (instance == nullptr) {
        instance = this;
    } else {
        Serial.println("Warning: Multiple BaseCommunicator instances detected!");
    }

    // Initialize LowLevelComm
    comm->init();

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");

    // Register callbacks
    esp_now_register_recv_cb(BaseCommunicator::onDataReceiveCallback);
    esp_now_register_send_cb(BaseCommunicator::onDataSendCallback);

    // Debug: Print MAX_FLAGS
    Serial.print("MAX_FLAGS is: ");
    Serial.println(MAX_FLAGS);
}

// Destructor
BaseCommunicator::~BaseCommunicator() {
    esp_now_deinit();
    instance = nullptr;
}

void BaseCommunicator::handleReceive(const esp_now_recv_info_t* recvInfo, const uint8_t *incomingData, int len) {
    const int headerOffset = 34; // 24 (MAC Header) + 1 (Category Code) + 3 (Org ID) + 4 (Random Values) + 2 (Element ID & Length)
    
    if (len <= headerOffset) {
        Serial.println("Error: Data length too short to contain payload.");
        return;
    }

    const uint8_t *payload = incomingData + headerOffset; // Skip headers
    int payloadLen = len - headerOffset;

    Serial.printf("Payload length: %d bytes\n", payloadLen);

    if (payloadLen == sizeof(ReceivedData)) {
        ReceivedData received;
        memcpy(&received, payload, sizeof(ReceivedData));

        if (received.flag >= 0 && received.flag < MAX_FLAGS) {
            if (msgMeasure == nullptr) msgMeasure = new ReceivedData;
            *msgMeasure = received;
            newMeasure = true;

            Serial.printf("Stored data for flag %d\n", received.flag);
        } else {
            Serial.println("Error: Invalid flag received.");
        }
    } else if (payloadLen == sizeof(ControlInput)) {
        if (msgCmd == nullptr) msgCmd = new ControlInput;
        memcpy(msgCmd, payload, sizeof(ControlInput));
        newMsgCmd = true;

        Serial.println("Received Control Command:");
        for (int i = 0; i < 13; i++) {
            Serial.print(msgCmd->params[i]);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("Error: Unknown payload size.");
    }
}

void BaseCommunicator::handleSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status to: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(mac_addr[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" | Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

// Static callback implementations
void BaseCommunicator::onDataReceiveCallback(const esp_now_recv_info_t* recvInfo, const uint8_t *incomingData, int len) {
    if (instance != nullptr) {
        instance->handleReceive(recvInfo, incomingData, len);
    }
}

void BaseCommunicator::onDataSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (instance != nullptr) {
        instance->handleSend(mac_addr, status);
    }
}

void BaseCommunicator::setMainBaseStation(){
    preferences.begin("params", true);

    if (preferences.getBytesLength("GroundMac") == 6) {
        Serial.print("Main Base Set: ");
        uint8_t mac_addr[6];
        preferences.getBytes("GroundMac", mac_addr, 6);
        memcpy(this->main_station_mac, mac_addr, 6); // Copy the MAC address
        for (int i = 0; i < 6; ++i) {
            if (i > 0) {
                Serial.print(":");
            }
            // Print each byte in hexadecimal format
            if (this->main_station_mac[i] < 16) {
                Serial.print("0"); // Print a leading zero for values less than 0x10
            }
            Serial.print(this->main_station_mac[i], HEX);
        }
        Serial.println();
        comm->addPeer(this->main_station_mac);
        this->addPingStation(this->main_station_mac);  // The main station is also a ping station.
    } else {
        Serial.println("Main Base Not Set.");
    }
    preferences.end(); 
}

void BaseCommunicator::addPingStation(const uint8_t mac_addr[6]) {
    if (numPingStations < MAX_PING_STATIONS) {
        memcpy(pStations[numPingStations], mac_addr, 6); // Add new ping station MAC address
        numPingStations++; // Increment the count of ping stations
        Serial.print("Added Ping Station: ");
        for (int i = 0; i < 6; i++) {
            Serial.print(pStations[numPingStations-1][i], HEX);
            if (i < 5) Serial.print(":");
        }
        Serial.println();
    } else {
        Serial.println("Error: Maximum number of ping stations reached.");
    }
}

void BaseCommunicator::pingStations() {
    for (int i = 0; i < numPingStations; ++i) {
        // Construct ping message
        const char* pingMessage = "PING";
        size_t pingMessageLength = strlen(pingMessage) + 1; // Include null terminator

        // Send ping
        comm->sendData(pStations[i], (const uint8_t*)pingMessage, pingMessageLength);
        Serial.print("Sent ping to: ");
        for (int j = 0; j < 6; j++) {
            Serial.print(pStations[i][j], HEX);
            if (j < 5) Serial.print(":");
        }
        Serial.println();
    }
}

bool BaseCommunicator::sendMeasurements(ReceivedData* measurements){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        // Save the last time you executed the code
        previousMillis = currentMillis;

        // Log the serialized data
        Serial.println("Serialized Data Sent:");
        uint8_t* data = (uint8_t*)measurements;
        for (int i = 0; i < sizeof(ReceivedData); i++) {
            Serial.printf("%02X ", data[i]); // Print each byte in hexadecimal
        }
        Serial.println();

        // Send data
        comm->sendData(this->main_station_mac, (uint8_t *) measurements, sizeof(ReceivedData));
        Serial.println("Sent measurements to main base station.");
        return true;
    }
    return false;  // Data not sent yet
}

bool BaseCommunicator::readNewMessages(){
    // Check for new messages
    if (newMeasure || newMsgCmd){
        return true;
    }
    return false;
}

ControlInput BaseCommunicator::receiveMsgCmd(){
    if (msgCmd != nullptr) {
        ControlInput cmd = *msgCmd;
        newMsgCmd = false;
        delete msgCmd;
        msgCmd = nullptr;
        return cmd;
    }
    ControlInput emptyCmd = {0};
    return emptyCmd;
}

bool BaseCommunicator::isNewMsgCmd(){
    return newMsgCmd;
}
