#ifndef MOTORS_H
#define MOTORS_H

#include "motor.h"
#include "definitions.h"

class Motors {
public:
    Motors() {};
    void init();
    void move(float speed, float direction, float rotation, float heading);

private:
    Motor motor[MOTOR_NUM] = {
        Motor(FL_INA, FL_INB, FL_PWM),
        Motor(FR_INA, FR_INB, FR_PWM),
        Motor(BR_INA, BR_INB, BR_PWM),
        Motor(BL_INA, BL_INB, BL_PWM)
    };

    float motorAngles[MOTOR_NUM] = {45.0f, 135.0f, 225.0f, 315.0f}; // left 0, cw
    float motorSpeeds[MOTOR_NUM] = {0.0f};
};

#endif