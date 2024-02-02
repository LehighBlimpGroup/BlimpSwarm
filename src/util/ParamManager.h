//
// Created by dav on 2/1/24.
//

#ifndef BLIMPSWARM_PARAMMANAGER_H
#define BLIMPSWARM_PARAMMANAGER_H


#include <Preferences.h>
#include <Arduino.h>
#include "util/data_types.h"

// Other robot types would be included here

class ParamManager {
    private:
        Preferences preferences;

        String readString(const uint8_t *data, int &index, uint8_t length);
    public:
        void parseAndSetPreference(const uint8_t* data, int data_len);
};



#endif //BLIMPSWARM_PARAMMANAGER_H
