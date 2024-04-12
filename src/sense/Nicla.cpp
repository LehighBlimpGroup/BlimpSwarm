//
// Created by eddie on 3/12/24.
//

#include "Nicla.h"

HardwareSerial MySerial0(0);
Nicla::Nicla() {
    for (int i = 0; i < 10; i++) {
        value[i] = 0;
    }
}

void Nicla::startup() {
    Nicla::getPreferences();
    Serial.println("Starting IBUS Init");
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);
    IBus.begin(MySerial0, IBUSBM_NOTIMER);
    IBus.loop();
    // mode = 0x80; //balloon detection
    //mode = 0x81; //goal detection
    MySerial0.write(mode); 
    sendTime = micros();
}

void Nicla::startup(uint8_t setMode) {
    Serial.println("Starting IBUS Init");
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);
    IBus.begin(MySerial0, IBUSBM_NOTIMER);
    IBus.loop();
    mode = setMode; 
    MySerial0.write(mode); 
    sendTime = micros();
}

//data = [flag, x_roi, y_roi, w_roi, h_roi, x_value, y_value, w_value, h_value, dis]
bool Nicla::update() {
    if (micros() - sendTime > 200000000) {// resend command every 5 seconds. 
        MySerial0.write(mode); 
        sendTime = micros();
    }
    IBus.loop();
    // check to see if the data is new
    // if (value[5] == (float)IBus.readChannel(5) && value[6] == (float)IBus.readChannel(6)) {
    //     return false;
    // } else if (IBus.readChannel(0) == 3) { // check to see if the nicla registers a detection
    //     return false;
    // }

    for (int i = 0; i < 10; i++){
        value[i] = (float)IBus.readChannel(i);
    }
    return true;
}

//data = [flag, x_roi, y_roi, w_roi, h_roi, x_value, y_value, w_value, h_value, dis]
bool Nicla::update(uint8_t setMode) {
    if (setMode != mode) {
        mode = setMode;
        MySerial0.write(mode); 
        sendTime = micros();
    }

    return Nicla::update();
    
}


float* Nicla::readValues(int& count) {
    count = 10; 
    return value;
}

void Nicla::getPreferences(){
    return;

}