//
// Created by dav on 1/22/24.
//

#ifndef BLIMPSWARM_LOWLEVELCOMM_H
#define BLIMPSWARM_LOWLEVELCOMM_H

#include "util/data_types.h"


class LowLevelComm {
public:
    // Network Initialization
    virtual void init() = 0;

    // Pure virtual function to send data
    virtual void sendData(const uint8_t* data, unsigned int length) = 0;

    // Pure virtual function to receive data
    virtual ControlInput receiveLongData() = 0;
    // True if new data arrived
    virtual bool newLongData() = 0;


    virtual ReceivedData receiveShortData() = 0;
    virtual bool newShortData() = 0;




};




#endif //BLIMPSWARM_LOWLEVELCOMM_H
