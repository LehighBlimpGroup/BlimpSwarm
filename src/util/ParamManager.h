//
// Created by dav on 2/1/24.
//

#ifndef BLIMPSWARM_PARAMMANAGER_H
#define BLIMPSWARM_PARAMMANAGER_H


#include <Preferences.h>
#include <Arduino.h>

// Other robot types would be included here

class ParamManager {
    private:
        enum DataType : uint8_t {
            DataType_Int = 0x01,
            DataType_Float = 0x02,
            DataType_String = 0x03,
            // Add more datatypes as needed
        };
        Preferences preferences;

        String readString(const uint8_t *data, int &index, uint8_t length);
    public:
        void parseAndSetPreference(const uint8_t* data, int data_len);
};



#endif //BLIMPSWARM_PARAMMANAGER_H
