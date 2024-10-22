#include <esp_now.h>
#include <WiFi.h>

uint8_t receiverAddress[] = {0xDC, 0x54, 0x75, 0xD8, 0x40, 0x74}; // Replace with the MAC address of ESP32 Board 2

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Sender ready");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  const char *message = "Hello from ESP32 Board 1";
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)message, strlen(message));

  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.println("Error sending message");
  }

  delay(2000); // Send every 2 seconds
}
