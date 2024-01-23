//
// Created by dav on 1/22/24.
//

#ifndef BLIMPSWARM_LOWLEVELCOMM_H
#define BLIMPSWARM_LOWLEVELCOMM_H



class LowLevelComm {
public:
    // Network Initialization
    virtual void init() = 0;

    // Pure virtual function to send data
    virtual void sendData(const uint8_t* data, unsigned int length) = 0;

    // Pure virtual function to receive data
    virtual void receiveData(const uint8_t* data, unsigned int length) = 0;

    // Pure virtual function to receive data
//    virtual bool newData() = 0;
};




#endif //BLIMPSWARM_LOWLEVELCOMM_H
