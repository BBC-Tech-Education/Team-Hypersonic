#include "camera.h"

void Camera::init() { // Initialises UART with baud rate
    CAMERA_SERIAL.begin(115200);
}

float Camera::distance(uint16_t cx, uint16_t cy) { // Finds pixel distance from center
    int16_t y = CY - cy;
    int16_t x = CX - cx;

    return sqrtf((float)(x*x + y*y));
}

float Camera::angle(uint16_t cx, uint16_t cy) { // Finds pixel angle from center
    int16_t y = CY - cy;
    int16_t x = CX - cx;

    float angle = 450.0f - (atan2f((float)y, (float)x) * RAD_TO_DEG_F); // Polar to Cartesian

    return floatMod(angle, 360.0f);
}

void Camera::update() {
    if (CAMERA_SERIAL.available() >= CAMERA_PACKET_NUMBER) { // Only update values when a full packet is available
        uint8_t first = CAMERA_SERIAL.read();
        uint8_t second = CAMERA_SERIAL.peek();

        if (first == CAMERA_START_BYTE && second == CAMERA_START_BYTE) { // Checks start bytes
            fps = (1000.0f)/(millis() - camTime);
            camTime = millis();

            CAMERA_SERIAL.read(); // Reads 2nd start byte

            for (uint8_t byte = 0; byte < CAMERA_PACKET_NUMBER - 2; byte++) { // Reads full packet
                receivedPacket[byte] = CAMERA_SERIAL.read();
            }

            for (uint8_t byte = 0; byte < (CAMERA_PACKET_NUMBER - 2) / 2; byte++) { // Converts byte pairs into 16-bit integers
                camData[byte] = receivedPacket[byte * 2] | (receivedPacket[(byte * 2) + 1] << 8);
            }

            if (camData[0] != 500) {
                lastTimeBallSeen = millis();
                float bPx = distance(camData[0], camData[1]);
                ballDist = (bPx == 0.0f) ? 0.0f : ballPixelToCm(fmaxf(bPx, 1.0f));
                ballAngle = angle(camData[0], camData[1]);
                ball = true;
            } else if ((millis() - lastTimeBallSeen) > 100) {
                ballDist = 0.0f;
                ballAngle = -1.0f;
                ball = false;
            }

            if (camData[2] != 500) {
                lastTimeYellowGoalSeen = millis();
                float yPx = distance(camData[2], camData[3]);
                yellowGoalDist = (yPx == 0.0f) ? 0.0f : goalPixelToCm(fmaxf(yPx, 1.0f));
                yellowGoalAngle = angle(camData[2], camData[3]);
                yellowGoal = true;
            } else if ((millis() - lastTimeYellowGoalSeen) > 100) {
                yellowGoalDist = 0.0f;
                yellowGoalAngle = -1.0f;
                yellowGoal = false;
            }

            if (camData[4] != 500) {
                lastTimeBlueGoalSeen = millis();
                float bPx = distance(camData[4], camData[5]);
                blueGoalDist = (bPx == 0.0f) ? 0.0f : goalPixelToCm(fmaxf(bPx, 1.0f));
                blueGoalAngle = angle(camData[4], camData[5]);
                blueGoal = true;
            } else if ((millis() - lastTimeBlueGoalSeen) > 100) {
                blueGoalDist = 0.0f;
                blueGoalAngle = -1.0f;
                blueGoal = false;
            }

            lastTimeAttackGoalSeen = BLUE_GOAL_ATTACK ? lastTimeBlueGoalSeen : lastTimeYellowGoalSeen;
            lastTimeDefendGoalSeen = BLUE_GOAL_ATTACK ? lastTimeYellowGoalSeen : lastTimeBlueGoalSeen;
            
            attackGoalDist = BLUE_GOAL_ATTACK ? blueGoalDist : yellowGoalDist;
            defendGoalDist = BLUE_GOAL_ATTACK ? yellowGoalDist : blueGoalDist;
            attackGoalAngle = BLUE_GOAL_ATTACK ? blueGoalAngle : yellowGoalAngle;
            defendGoalAngle = BLUE_GOAL_ATTACK ? yellowGoalAngle : blueGoalAngle;

            attackGoal = BLUE_GOAL_ATTACK ? (blueGoalDist != 0.0f) : (yellowGoalDist != 0.0f);
            defendGoal = BLUE_GOAL_ATTACK ? (yellowGoalDist != 0.0f) : (blueGoalDist != 0.0f);
        }
    }
}
