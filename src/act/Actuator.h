/**
 * @file Actuator.h
 * @author David Saldana
 * @brief Parent class for actuation of robot components
 * @version 0.1
 * @date 2024-02-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef BLIMPSWARM_ACTUATOR_H
#define BLIMPSWARM_ACTUATOR_H

class Actuator {
    public:
        Actuator(int pinVal) : min(0), max(1), offset(0), pin(pinVal){
        }

        Actuator(int minVal, int maxVal, int offsetVal, int pinVal)
                : min(minVal), max(maxVal), offset(offsetVal), pin(pinVal){
        }

        // Setter method for offset
        void setOffset(int newOffset) {
            offset = newOffset;
        }

        virtual void act(float value) = 0;

    protected:
        int min;
        int max;
        int offset;
        int pin;
};


#endif //BLIMPSWARM_ACTUATOR_H
