#ifndef LS_H
#define LS_H

#include "config.h"
#include "definitions.h"

class LightSensors {
    public:
        LightSensors() {};
        void init();
        void read();
        void update(float heading);

        float lineSize;
        float lineDir;

        float fieldLineAngle = -1.0f;
        float fieldLineSize = -1.0f;

    private:
        uint8_t lightPins[4] = {LS_0, LS_1, LS_2, LS_3};
        uint8_t lightOutPins[3] = {LS_OUT_1, LS_OUT_2, LS_OUT_3};

        uint8_t muxIndex[LS_NUM] = {
        //   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
            25, 26, 27, 28, 29, 30, 31,  0,  8,  7,  6,  5,  4,  3,  2,  1, // mux 1
             9, 10, 11, 12, 13, 14, 15, 16, 24, 23, 22, 21, 20, 19, 18, 17, // mux 2
            34, 35, 36, 37, 38, 39, 40, 41, 33, 32, 47, 46, 45, 44, 43, 42  // mux 3
        };

        uint8_t clusterNum;
        bool inCluster;

        struct Cluster {
            int8_t start = -1;
            float midpoint = -1.0f;
            int8_t end = -1;
        };

        uint16_t lightValues[LS_NUM] = {0};
        uint16_t whiteCal[LS_NUM] = {0}; 
        uint8_t whiteValues[LS_NUM] = {0};

        bool onField = true;

};

#endif