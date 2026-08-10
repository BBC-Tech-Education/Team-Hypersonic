#ifndef VD_H
#define VD_H

#include "definitions.h"

class VoltageDivider {
    public:
        VoltageDivider() {}
        
        void init();
        void update(); 

        bool batteryLow = false;
        
    private:
        uint8_t counter = 0;
        
        
};

#endif