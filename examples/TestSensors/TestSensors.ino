#include <Arduino.h>
#include "BlimpSwarm.h"
#include "sense/NiclaSuite.h"

NiclaSuite sensors; 

void setup(){
  delay(250);
  Serial.begin(115200);
  Serial.println("Start Sensortest");
  sensors.startup();
  Serial.println("setup done");
}

void loop(){
  sensors.update();
  int test;
  float* sensorValues = sensors.readValues(test);

  for (int i = 0; i < test; i++){
    Serial.print(sensorValues[i]);
    if(i != test-1)
      Serial.print(" : ");
  }
  Serial.println();
  delay(20);

}