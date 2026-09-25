#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "config.h"
#include "definitions.h"

class Bluetooth {
    public:
        Bluetooth() {};

        void init();
        void update(float absoluteBallAngle, float ballDist);

        bool attack = ATTACK;

    private:
        void send(float absoluteBallAngle, float ballDist);
        void receive();
        void resolveConflict();

        bool switching = false;
        bool connected = false;

        uint8_t receivedPacket[5] = {0};
        uint16_t otherData[3] = {0};
        uint8_t counter = 0;

        unsigned long lastTimeConnected = 0;
        unsigned long lastTimeSent = millis();
        unsigned long lastTimeSwitched = millis();

};

#endif