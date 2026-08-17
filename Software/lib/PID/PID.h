#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    PID(float p, float i, float d, float absoluteMax = 0.0f);
    float update(float input, float setpoint, float modulus = 0.0f);

private:
    float kp;
    float ki;
    float kd;
    unsigned long lastTime; // = micros()? will break stuff

    float absMax;
    float integral = 0.0f;
    float lastError = 0.0f;
};

#endif