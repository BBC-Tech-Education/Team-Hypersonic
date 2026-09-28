#include "bluetooth.h"

void Bluetooth::init() { // Initialises UART with baud rate
    BLUETOOTH_SERIAL.begin(38400);
}

void Bluetooth::send(float absoluteBallAngle, float ballDist, float posX, float posY) { // Sends ball distance and role
    BLUETOOTH_SERIAL.write(BLUETOOTH_START_BYTE);
    BLUETOOTH_SERIAL.write(BLUETOOTH_START_BYTE);
    BLUETOOTH_SERIAL.write(((uint16_t)absoluteBallAngle) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)absoluteBallAngle) >> 8) & 255); // Upper half
    BLUETOOTH_SERIAL.write(((uint16_t)ballDist) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)ballDist) >> 8) & 255); // Upper half
    BLUETOOTH_SERIAL.write(((uint16_t)posX) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)posX) >> 8) & 255); // Upper half
    BLUETOOTH_SERIAL.write(((uint16_t)posY) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)posY) >> 8) & 255); // Upper half
    BLUETOOTH_SERIAL.write(attack);
    lastTimeSent = millis();
}

void Bluetooth::receive() {
    if (BLUETOOTH_SERIAL.available() >= BLUETOOTH_PACKET_NUMBER) {
        uint8_t first = BLUETOOTH_SERIAL.read();
        uint8_t second = BLUETOOTH_SERIAL.peek();

        if ((first == BLUETOOTH_START_BYTE) && (second == BLUETOOTH_START_BYTE)) {
            lastTimeConnected = millis();

            BLUETOOTH_SERIAL.read();

            for (uint8_t byte = 0; byte < BLUETOOTH_PACKET_NUMBER - 2; byte++) { // Reads full packet
                receivedPacket[byte] = BLUETOOTH_SERIAL.read();
            }

            otherData[0] = receivedPacket[0] | (receivedPacket[1] << 8); // Recombines to a 16 bit number
            otherData[1] = receivedPacket[2] | (receivedPacket[3] << 8); // Recombines to a 16 bit number
            otherData[2] = receivedPacket[4] | (receivedPacket[5] << 8); // Recombines to a 16 bit number
            otherData[3] = receivedPacket[6] | (receivedPacket[7] << 8); // Recombines to a 16 bit number
            otherData[4] = receivedPacket[8];
        }
    }
}

void Bluetooth::resolveConflict() {
    if (counter > 100) {
        if (attack != ATTACK) { // MAYBE CHANGE IT TO GO TO DEFENDER IF CONNECTION IS STABLE
            attack = ATTACK; // reverts back to original role
            lastTimeSwitched = millis();
        }
    }

    if ((uint8_t)(((attack + otherData[4]) == 0) || ((attack + otherData[4]) == 2))) counter ++;
    else counter = 0;
}

void Bluetooth::update(float absoluteBallAngle, float ballDist, float posX, float posY) {
    if (!isfinite(posX)) posX = 0.0f;
    if (!isfinite(posY)) posY = 0.0f;

    posX = constrain(posX, -1000.0f, 1000.0f);
    posY = constrain(posY, -1000.0f, 1000.0f);

    receive();
    connected = (millis() - lastTimeConnected < 500); // 0.5s

    if (connected) {
        if (millis() - lastTimeSent > 50) send(absoluteBallAngle, ballDist, posX, posY);

        if (!attack) { // Defender
            if (millis() - lastTimeSwitched > 1000) {
                if ((smallestAngleBetween(absoluteBallAngle, 0.0f) <= 90.0f) && (ballDist <= 20.0f)) {
                    attack = true; // switches
                    lastTimeSwitched = millis();
                }
            }

        } else { // Attacker
            if (otherData[4] == 1) { // defender switched to attacker
                attack = false; // switches to defender
                lastTimeSwitched = millis();
            }
        }

        resolveConflict();   
    } else {
        if (attack != ATTACK) { // MAYBE CHANGE IT TO GO TO DEFENDER IF CONNECTION IS STABLE
            attack = ATTACK; // reverts back to original role
            lastTimeSwitched = millis();
        }
        counter = 0;
    }
}