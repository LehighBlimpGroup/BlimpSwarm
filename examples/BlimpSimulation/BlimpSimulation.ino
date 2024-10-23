// ESP32 Simulated Blimp Code

#include <esp_now.h>
#include <WiFi.h>

// Structure for control parameters
typedef struct ControlParams {
  uint8_t mac[6];
  float params[13];
} ControlParams;

// Structure for sensor data
typedef struct SensorData {
  float data[6]; 
} SensorData;

// Replace with the MAC address of the laptop bridge ESP32
uint8_t laptopMac[6] = {0x30, 0x30, 0xF9, 0x34, 0x68, 0x94}; 

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void sendSensorData();

unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Simulated Blimp Ready");

  // Print MAC address
  Serial.print("Blimp ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Add laptop bridge as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, laptopMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Laptop bridge peer added");
  } else {
    Serial.println("Failed to add laptop bridge peer");
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    sendSensorData();
    lastSendTime = currentTime;
  }
}

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t *data, int data_len) {
  // Process control parameters
  if (data_len == sizeof(ControlParams)) {
    ControlParams controlParams;
    memcpy(&controlParams, data, sizeof(ControlParams));
    Serial.println("Received control parameters");

    Serial.print("Sender MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X:", recv_info->src_addr[i]);
    }
    Serial.println();
  }
}

void sendSensorData() {
  SensorData sensorData;
  // Simulate sensor data
  sensorData.data[0] = random(0, 100) / 10.0; // Height
  sensorData.data[1] = random(0, 360);        // Yaw
  sensorData.data[2] = random(0, 1024);       // Sensor1
  sensorData.data[3] = random(0, 1024);       // Sensor2
  sensorData.data[4] = random(0, 1024);       // Sensor3
  sensorData.data[5] = random(0, 100);        // Battery

  // Send sensor data back to laptop via ESP-NOW
  esp_err_t result = esp_now_send(laptopMac, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result != ESP_OK) {
    Serial.println("Error sending sensor data");
  }
}
