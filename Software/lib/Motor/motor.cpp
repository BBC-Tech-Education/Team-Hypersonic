#include "motor.h"

Motor::Motor(uint8_t ina, uint8_t inb, uint8_t pwm) { // Constructor for Motor class
    INA = ina;
    INB = inb;
    PWM = pwm;
}

void Motor::init() {
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(PWM, OUTPUT);

    // Starts up motor controllers properly to avoid going into latch state 
    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH);
    delayMicroseconds(100);
    analogWriteFrequency(PWM, MOTOR_PWM_FREQ); // 15000 less than recommended maximum of 20000 from datasheet
    analogWrite(PWM, 0);
}

void Motor::move(float speed) {
    uint8_t pwm = (uint8_t)(fabsf(speed / 100.0f) * 255.0f); // Scales 0-100 to 0-255
    uint8_t ina = (speed > 0.0f) ? HIGH : LOW;
    uint8_t inb = (speed < 0.0f) ? HIGH : LOW;

    digitalWrite(INA, ina);
    digitalWrite(INB, inb);
    analogWrite(PWM, pwm);
}