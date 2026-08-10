#ifndef MOTOR_H
#define MOTOR_H

#include "definitions.h"

class Motor {
public:
    Motor(uint8_t ina, uint8_t inb, uint8_t pwm);
    void init();
    void move(float speed);

private:
    uint8_t INA, INB, PWM;
};

#endif