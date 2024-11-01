#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

void onReceive(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  Serial.print("Received data from: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(recvInfo->src_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" | Data: ");
  for (int i = 0; i < len / sizeof(float); i++) {
    Serial.print(((float*)incomingData)[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  esp_now_register_recv_cb(onReceive);
}

void loop() {
  // No need for loop code as the onReceive callback handles data reception
}
