#include "camera.h"

 /*
 TODO:
 -> check distances and angles are correct
 -> tune goal pixel to cm function
 */

void Camera::init() { // Initialise UART with baud rate
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
            // fps = (1000.0f)/(millis() - camTime);
            // camTime = millis();
            // Serial.println(fps);

            CAMERA_SERIAL.read(); // Reads 2nd start byte

            for (uint8_t byte = 0; byte < CAMERA_PACKET_NUMBER - 2; byte++) { // Reads full packet
                receivedPacket[byte] = CAMERA_SERIAL.read();
                // Serial.print(receivedPacket[byte]);
                // Serial.print(" ");
            }
            // Serial.println();

            for (uint8_t byte = 0; byte < (CAMERA_PACKET_NUMBER - 2) / 2; byte++) { // Converts byte pairs into 16-bit integers
                camData[byte] = receivedPacket[byte * 2] | (receivedPacket[(byte * 2) + 1] << 8);
                // Serial.print(camData[byte]);
                // Serial.print(" ");
            }
            // Serial.println();

            ballDist = (camData[0] == 500) ? 0.0f : ballPixelToCm(distance(camData[0], camData[1])); // 500.0f means no ball detected
            ballAngle = (camData[0] == 500) ? -1.0f : angle(camData[0], camData[1]);
            float yellowGoalDist = (camData[2] == 500) ? 0.0f : goalPixelToCm(distance(camData[2], camData[3]));
            float yellowGoalAngle = (camData[2] == 500) ? -1.0f : angle(camData[2], camData[3]);
            float blueGoalDist = (camData[4] == 500) ? 0.0f : goalPixelToCm(distance(camData[4], camData[5]));
            float blueGoalAngle = (camData[4] == 500) ? -1.0f : angle(camData[4], camData[5]);
            attackGoalDist = BLUE_GOAL_ATTACK ? blueGoalDist : yellowGoalDist;
            defendGoalDist = BLUE_GOAL_ATTACK ? yellowGoalDist : blueGoalDist;
            attackGoalAngle = BLUE_GOAL_ATTACK ? blueGoalAngle : yellowGoalAngle;
            defendGoalAngle = BLUE_GOAL_ATTACK ? yellowGoalAngle : blueGoalAngle;
        }
    }
}