#include "voltageDivider.h"

void VoltageDivider::init() { // Initialisation
    pinMode(VOLTAGE_DIVIDER_PIN, INPUT);
}

void VoltageDivider::update() {
    battVoltage = analogRead(VOLTAGE_DIVIDER_PIN) * SCALE_FACTOR; // Converts voltdiv voltage to real voltage

    if (counter > 100) { // has to be under 11.1V for 100 consecutive loops
        batteryLow = true;
        return;
    }

    if (battVoltage < LOW_BATTERY_VOLTAGE) counter ++;
    else counter = 0;
    
    batteryLow = false;
}