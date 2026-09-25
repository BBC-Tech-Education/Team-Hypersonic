#include "bluetooth.h"

void Bluetooth::init() { // Initialises UART with baud rate
    BLUETOOTH_SERIAL.begin(38400);
}

void Bluetooth::send(float absoluteBallAngle, float ballDist) { // Sends ball distance and role
    BLUETOOTH_SERIAL.write(BLUETOOTH_START_BYTE);
    BLUETOOTH_SERIAL.write(BLUETOOTH_START_BYTE);
    BLUETOOTH_SERIAL.write(((uint16_t)absoluteBallAngle) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)absoluteBallAngle) >> 8) & 255); // Upper half
    BLUETOOTH_SERIAL.write(((uint16_t)ballDist) & 255); // Lower half
    BLUETOOTH_SERIAL.write((((uint16_t)ballDist) >> 8) & 255); // Upper half
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
            otherData[2] = receivedPacket[4];
        }
    }
}

void Bluetooth::resolveConflict() {
    if (counter > 100) {
        attack = ATTACK; // reverts back to original role
    }

    if ((uint8_t)((attack + otherData[2] == 0) || (attack + otherData[2] == 2))) counter ++;
    else counter = 0;
}

void Bluetooth::update(float absoluteBallAngle, float ballDist) {
    receive();
    connected = (millis() - lastTimeConnected < 500);

    if (!attack) { // Defender

        if (millis() - lastTimeSwitched < 1000) {
            return;
        }

        if ((smallestAngleBetween(absoluteBallAngle, 0.0f) <= 90.0f) && (ballDist <= 20.0f)) {
            attack = true; // switches
        }

    } else { // Attacker
        if (otherData[2] == 1) { // defender switched to attacker
            attack = false; // switches to defender
        }
    }

    if (millis() - lastTimeSent > 50) { // 20 Hz to avoid overloading buffer
        send(absoluteBallAngle, ballDist);
    }

    resolveConflict();    
}