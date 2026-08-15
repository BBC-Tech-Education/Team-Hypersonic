#ifndef VOLTDIV_H
#define VOLTDIV_H

#include "definitions.h"

class VoltageDivider {
    public:
        VoltageDivider() {}
        
        void init();
        void update(); 

        bool batteryLow = false;

        float battVoltage = -1.0f;
        
    private:
        uint8_t counter = 0;
        
        
};

#endif