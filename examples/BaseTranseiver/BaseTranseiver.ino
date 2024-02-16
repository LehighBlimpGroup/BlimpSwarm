#include <esp_now.h>
#include <WiFi.h>

// Adjust these values as needed for your specific application
constexpr size_t CONTROL_PARAMS_SIZE = sizeof(float) * 13; // Size of control parameters
constexpr size_t MAC_ADDRESS_SIZE = 6; // MAC address size

// Structure to hold control parameters
struct ControlParams {
  float params[13]; // Example: 13 control parameters
};
const int MAX_RECEIVERS = 15;      
const int MAC_LENGTH = 6;          

uint8_t slaveAddresses[MAX_RECEIVERS][MAC_LENGTH] = {};

// Function declarations
void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void onDataSend(const uint8_t *mac, esp_now_send_status_t status);
void addPeer(const uint8_t *peerAddr);
void removePeer(const uint8_t *peerAddr);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(200);
  Serial.print("Transmitter ESP Board");
  // Serial.print(WiFi.macAddress());

  esp_now_deinit();
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSend);
  esp_now_register_recv_cb(onDataReceive);
}

void loop() {
  // Listening for incoming serial data indicating MAC addresses to add/remove peers
  while (Serial.available()) {
    char type = Serial.read(); // Read the type of operation
    uint8_t peerAddr[MAC_ADDRESS_SIZE];
    if (type == 'A') { // Add peer
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        addPeer(peerAddr);
      }
    } else if (type == 'R') { // Remove peer
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        removePeer(peerAddr);
      }
    } else if (type == 'C') { // Control command
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        ControlParams params;
        if (Serial.readBytes((char*)&params, CONTROL_PARAMS_SIZE) == CONTROL_PARAMS_SIZE) {
          esp_now_send(peerAddr, (uint8_t*)&params, CONTROL_PARAMS_SIZE);
        }
        Serial.println("Sent Controls!");
      }
    } else if (type == 'D') { // Preference to save
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        static uint8_t buffer[250]; // Adjust based on your expected maximum message size
        size_t bytesToRead = Serial.available(); // Determine how many bytes we have for the preference data
        bytesToRead = min(bytesToRead, sizeof(buffer)); // Ensure we don't read more than the buffer size
        
        size_t bytesRead = Serial.readBytes(buffer, bytesToRead); // Read the preference data into the buffer
        Serial.println("Sent Preferences! ");
        // Serial.println((char*)buffer);
        esp_now_send(peerAddr, buffer, bytesRead); // Send the preference data to the specified peer
      }
    }
  }
}
esp_now_peer_info_t peerInfo;

void addPeer(const uint8_t *peerAddr) {
  memcpy(peerInfo.peer_addr, peerAddr, MAC_ADDRESS_SIZE);
  peerInfo.channel = 0; // Use auto channel
  peerInfo.encrypt = false; // No encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Added peer successfully.");
  }
}

void removePeer(const uint8_t *peerAddr) {
  if (esp_now_del_peer(peerAddr) != ESP_OK) {
    Serial.println("Failed to remove peer");
  }
}

void onDataSend(const uint8_t *mac, esp_now_send_status_t status) {
  // char ackMessage[32]; // Buffer for the acknowledgement message
  // if (status == ESP_NOW_SEND_SUCCESS) {
  //   snprintf(ackMessage, sizeof(ackMessage), "Send OK to %02x:%02x:%02x:%02x:%02x:%02x\n",
  //            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // } else {
  //   snprintf(ackMessage, sizeof(ackMessage), "Send FAIL to %02x:%02x:%02x:%02x:%02x:%02x\n",
  //            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // }
  // Serial.write(ackMessage);
}

void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Handle received data, if necessary
}
