/**
 * @file BaseTransceiver.ino
 * @brief Receiver and Transceiver for ESP-NOW communication using BlimpSwarm library
 * @version 1.1
 * @date 2024-04-27
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BlimpSwarm.h> // Include the main BlimpSwarm library header

#define MAX_RECEIVERS 15
#define MAC_LENGTH 6

// Ensure that MAX_FLAGS is defined in DataTypes.h
#ifndef MAX_FLAGS
#define MAX_FLAGS 6
#endif

float storedData[MAX_FLAGS][6] = {0};    // Data storage
bool flagReceived[MAX_FLAGS] = {false};  // Keep track of received flags

// Instance of ReceivedData to store the latest received data
ReceivedData latestReceivedData;

// Array to store peers' MAC addresses
uint8_t peers[MAX_RECEIVERS][MAC_LENGTH] = {};
int numPeersAdded = 0;

// ESP-NOW peer information
esp_now_peer_info_t peerInfo;

// Function declarations
void onDataReceive(const esp_now_recv_info *info, const uint8_t *data, int len);
void onDataSend(const uint8_t *mac, esp_now_send_status_t status);
void addPeer(const uint8_t *peerAddr);
void removePeer(const uint8_t *peerAddr);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(200);
  
  Serial.println("Transceiver ESP Board Initialized");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW initialized successfully");
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSend);
  esp_now_register_recv_cb(onDataReceive);
}

void loop() {
  // Listening for incoming serial data indicating operations
  while (Serial.available()) {
    char type = Serial.read(); // Read the type of operation
    uint8_t peerAddr[MAC_LENGTH];
    
    if (type == 'A') { // Add peer
      if (Serial.readBytes((char*)peerAddr, MAC_LENGTH) == MAC_LENGTH) {
        addPeer(peerAddr);
      }
    } 
    else if (type == 'R') { // Remove peer
      if (Serial.readBytes((char*)peerAddr, MAC_LENGTH) == MAC_LENGTH) {
        removePeer(peerAddr);
      }
    } 
    else if (type == 'C') { // Control command
      // Ensure ControlInput structure is consistent with sender
      ControlInput params;
      if (Serial.readBytes((char*)&params, sizeof(ControlInput)) == sizeof(ControlInput)) {
        // Send to all peers
        for (int i = 0; i < numPeersAdded; i++) {
          esp_now_send(peers[i], (uint8_t*)&params, sizeof(ControlInput));
        }
        Serial.println("Sent Control Command to all peers.");
      }
    } 
    else if (type == 'D') { // Preference to save
      if (Serial.readBytes((char*)peerAddr, MAC_LENGTH) == MAC_LENGTH) {
        static uint8_t buffer[250]; // Adjust based on your expected maximum message size
        size_t bytesToRead = Serial.available(); // Determine how many bytes we have for the preference data
        bytesToRead = min(bytesToRead, sizeof(buffer)); // Ensure we don't read more than the buffer size
        
        size_t bytesRead = Serial.readBytes((char*)buffer, bytesToRead); // Read the preference data into the buffer
        Serial.println("Sending Preferences...");
        esp_now_send(peerAddr, buffer, bytesRead); // Send the preference data to the specified peer
      }
    } 
    else if (type == 'G') { // Add ground station to drone for feedback
      if (Serial.readBytes((char*)peerAddr, MAC_LENGTH) == MAC_LENGTH) {
        uint8_t groundStationMAC[MAC_LENGTH + 1]; // Array to hold the MAC address plus the prefix byte
        groundStationMAC[0] = 0x70; // Set the first byte to 0x70 as required
        
        // Use WiFi.macAddress() to get the MAC address and store it directly into the array starting from the second position
        WiFi.macAddress(&groundStationMAC[1]); // Fill the array with the MAC address starting from position 1
        
        // Now groundStationMAC contains the prefixed MAC address ready to be sent
        esp_now_send(peerAddr, groundStationMAC, sizeof(groundStationMAC)); // Send the ground station MAC with the prefix
        
        Serial.println("Sent Ground Station MAC.");
      }
    } 
    else if (type == 'I') { // Retrieve data from ground station to use in Python
      // Example: Retrieve data for flag 1
      if (flagReceived[1]) {
        for (int i = 0; i < 6; i++) {
          Serial.write((uint8_t*)&storedData[1][i], sizeof(storedData[1][i]));
        }
        Serial.println(); // End of transmission for easier reading on Python side
        flagReceived[1] = false; // Reset the flag after reading
      } else {
        Serial.println("No Data Available for Flag 1.");
      }
    }
    else {
      Serial.println("Unknown Command Received.");
    }
  }
}

/**
 * @brief Callback function when data is sent
 * 
 * @param mac_addr MAC address of the recipient
 * @param status Status of the send operation
 */
void onDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status to: ");
  for (int i = 0; i < MAC_LENGTH; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < MAC_LENGTH - 1) Serial.print(":");
  }
  Serial.print(" | Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

/**
 * @brief Callback function when data is received
 * 
 * @param info Pointer to information about the received data
 * @param data Pointer to the received data
 * @param len Length of the received data
 */
void onDataReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  const uint8_t* mac_addr = info->src_addr; // Extract sender's MAC address

  // Print the sender's MAC address
  Serial.print("Data received from: ");
  for (int i = 0; i < MAC_LENGTH; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < MAC_LENGTH - 1) Serial.print(":");
  }
  Serial.println();

  // Print the received data length
  Serial.printf("Data length: %d bytes\n", len);

  // Print raw data in hexadecimal
  Serial.print("Raw Data: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();

  // Determine the type of data based on length
  if (len == sizeof(ReceivedData)) {
    // Received ReceivedData structure
    memcpy(&latestReceivedData, data, len);

    // Print the received flag and values
    Serial.printf("Flag: %d\n", latestReceivedData.flag);
    Serial.print("Values: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%.2f ", latestReceivedData.values[i]);
    }
    Serial.println();

    // Validate the received flag
    if (latestReceivedData.flag >= 0 && latestReceivedData.flag < MAX_FLAGS) {
      // Store the received values
      for (int i = 0; i < 6; i++) {
        storedData[latestReceivedData.flag][i] = latestReceivedData.values[i];
      }
      flagReceived[latestReceivedData.flag] = true;  // Mark this flag as received

      // Indicate successful processing
      Serial.println("Data processed and stored successfully.");
    } else {
      // Invalid flag received
      Serial.println("Error: Invalid flag received.");
    }
  }
  else if (len == sizeof(ControlInput)) {
    // Received ControlInput structure
    ControlInput receivedControl;
    memcpy(&receivedControl, data, len);

    // Print the received control parameters
    Serial.println("Received Control Command:");
    for (int i = 0; i < 13; i++) {
      Serial.printf("%.2f ", receivedControl.params[i]);
    }
    Serial.println();

    // TODO: Handle ControlInput as needed
  }
  else {
    // Data length mismatch
    Serial.println("Error: Received data size does not match the expected structure.");
  }

  // Print a separator for readability
  Serial.println("--------------------");
}

/**
 * @brief Adds a peer to the ESP-NOW peer list
 * 
 * @param peerAddr MAC address of the peer to add
 */
void addPeer(const uint8_t *peerAddr) {
  if (numPeersAdded >= MAX_RECEIVERS) {
    Serial.println("Error: Maximum number of peers reached.");
    return;
  }

  // Check if peer already exists
  bool exists = false;
  for (int i = 0; i < numPeersAdded; i++) {
    if (memcmp(peers[i], peerAddr, MAC_LENGTH) == 0) {
      exists = true;
      break;
    }
  }

  if (exists) {
    Serial.println("Peer already exists.");
    return;
  }

  // Add the peer to the array
  memcpy(peers[numPeersAdded], peerAddr, MAC_LENGTH);
  numPeersAdded++;

  // Add the peer to ESP-NOW
  memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize peerInfo
  memcpy(peerInfo.peer_addr, peerAddr, MAC_LENGTH);
  peerInfo.channel = 0;       // Use the current channel
  peerInfo.encrypt = false;   // No encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Added peer successfully.");
  }
}

/**
 * @brief Removes a peer from the ESP-NOW peer list
 * 
 * @param peerAddr MAC address of the peer to remove
 */
void removePeer(const uint8_t *peerAddr) {
  // Find the peer in the array
  int index = -1;
  for (int i = 0; i < numPeersAdded; i++) {
    if (memcmp(peers[i], peerAddr, MAC_LENGTH) == 0) {
      index = i;
      break;
    }
  }

  if (index == -1) {
    Serial.println("Peer not found.");
    return;
  }

  // Remove the peer from the array by shifting others
  for (int i = index; i < numPeersAdded - 1; i++) {
    memcpy(peers[i], peers[i + 1], MAC_LENGTH);
  }
  numPeersAdded--;

  // Remove the peer from ESP-NOW
  if (esp_now_del_peer(peerAddr) != ESP_OK) {
    Serial.println("Failed to remove peer");
  } else {
    Serial.println("Removed peer successfully.");
  }
}
