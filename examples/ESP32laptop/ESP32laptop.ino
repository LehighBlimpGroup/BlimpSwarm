// ESP32 Laptop Bridge Code

#include <esp_now.h>
#include <WiFi.h>

#define SERIAL_BAUDRATE 115200

// Define the blimp's MAC address
uint8_t blimpMac[6] = {0xDC, 0x54, 0x75, 0xD8, 0x40, 0x74}; // MAC address of the simulated blimp

// Structure for sensor data
typedef struct SensorData {
  float data[6]; // Adjust size as needed
} SensorData;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  WiFi.mode(WIFI_STA);
  Serial.println("ESP32 Laptop Bridge Ready");

  // Print MAC address of this ESP32
  Serial.print("Laptop Bridge ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Add the blimp as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, blimpMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Blimp peer added");
  } else {
    Serial.println("Failed to add blimp peer");
  }
}

void loop() {
  handleSerialCommands();
}

void handleSerialCommands() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'C') {
      // Control command
      uint8_t data[6 + 13 * 4]; // MAC (6 bytes) + 13 floats (4 bytes each)
      Serial.readBytes(data, sizeof(data));
      // Send via ESP-NOW to blimp
      esp_err_t result = esp_now_send(blimpMac, data, sizeof(data));
      if (result == ESP_OK) {
        Serial.println("Control command sent");
      } else {
        Serial.println("Error sending control command");
      }
    } else if (command == 'I') {
    }
  }
}

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t *data, int data_len) {
  // Forward sensor data to Serial
  if (data_len == sizeof(SensorData)) {
    SensorData sensorData;
    memcpy(&sensorData, data, sizeof(SensorData));
    // Send sensor data to Serial as a line
    Serial.print("SENSOR_DATA:");
    for (int i = 0; i < 6; i++) {
      Serial.print(sensorData.data[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.println();

    Serial.print("Sender MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X:", recv_info->src_addr[i]);
    }
    Serial.println();
  }
}
