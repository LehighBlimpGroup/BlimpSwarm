#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "BlimpSwarm.h"
#include "sense/SensorSuite.h"

NiclaSuite sensors;
uint8_t receiverMAC[] = {0xF0, 0xF5, 0xBD, 0x2C, 0xFD, 0xDC};

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send Success" : "Send Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  sensors.startup();
}

void loop() {
  sensors.update();
  int sensorCount;
  float* sensorValues = sensors.readValues(sensorCount);
  esp_now_send(receiverMAC, (uint8_t *)sensorValues, sizeof(float) * sensorCount);
  delay(10);
}
