#include <esp_now.h>
#include <WiFi.h>

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  char incomingMessage[250];
  memcpy(incomingMessage, data, data_len);
  incomingMessage[data_len] = '\0'; // Null-terminate the string

  Serial.print("Received message from: ");
  const uint8_t *mac_addr = recv_info->src_addr;
  for (int i = 0; i < 6; ++i) {
    if (mac_addr[i] < 16) Serial.print("0");  // Add leading zero if needed
    Serial.print(mac_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" - ");
  Serial.println(incomingMessage);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Receiver ready");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Do nothing in loop
}
