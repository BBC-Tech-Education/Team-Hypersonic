#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "config.h"
#include "definitions.h"

class Bluetooth {
    public:
        Bluetooth() {};

        void init();
        void update(float absoluteBallAngle, float ballDist, float posX, float posY);

        bool attack = ATTACK;

    private:
        void send(float absoluteBallAngle, float ballDist, float posX, float posY);
        void receive();
        void resolveConflict();

        bool switching = false;
        bool connected = false;

        uint8_t receivedPacket[BLUETOOTH_PACKET_NUMBER - 2] = {0};
        uint16_t otherData[5] = {0};
        uint8_t counter = 0;

        unsigned long lastTimeConnected = 0;
        unsigned long lastTimeSent = millis();
        unsigned long lastTimeSwitched = millis();

};

#endif