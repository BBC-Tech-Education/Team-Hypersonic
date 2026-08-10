#include "motors.h"

/*
TODO:
-> actually check if movement in every direction is accurate
-> check motorAngles
*/

void Motors::init() { // Initialisation
    for (uint8_t i = 0; i < MOTOR_NUM; i++) { 
        motor[i].init();
    }
}

void Motors::move(float speed, float direction, float rotation, float heading) {
    float highest = -1.0f;
    float dir = floatMod(direction - heading, 360.0f); // relative direction

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        motorSpeeds[i] = -speed * cosf((motorAngles[i] - dir) * DEG_TO_RAD_F) - rotation; // Vector projection of movement vector onto motor vector
        highest = fmaxf(highest, fabsf(motorSpeeds[i]));
    }

    if (highest > 100.0f) { // Scales motor speeds down if speed above 100
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            motorSpeeds[i] *= 100.0f/highest;
        }
    }
    
    // for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    //     Serial.print(motorSpeeds[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        motor[i].move(constrain(motorSpeeds[i], -100.0f, 100.0f));
    }
}