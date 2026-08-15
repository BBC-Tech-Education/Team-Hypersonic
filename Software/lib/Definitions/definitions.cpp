#include "definitions.h"

int16_t sign(int16_t value) { // Returns positive or negative (integer)
    return (value >= 0) ? 1 : -1;
}

float fsign(float value) { // Returns positive or negative (float)
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

float floatMod(float x, float m) { // Normalises angle
    float r = fmodf(x, m);
    return r < 0.0f ? r + m : r;
}

float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleClockwise - angleCounterClockwise, 360.0f);
}
  
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float angle = angleBetween(angleCounterClockwise, angleClockwise);
    return fminf(angle, 360.0f - angle);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0f, 360.0f);
}

int16_t mod(int16_t x, int16_t m) {
    int16_t r = x % m;
    return r < 0 ? r + m : r;
}

bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck) {
    if (angleBoundCounterClockwise < angleBoundClockwise) {
        return (angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
    } else {
        return (angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
    }
}

float ballPixelToCm(float ballPixelDist) {
    #if ROBOT_1
    return (2.33681f * expf(0.0180409f * ballPixelDist)) - 2.33681f;
    #else 
    return 0.00304002f * ballPixelDist * ballPixelDist;
    #endif
}

float goalPixelToCm(float goalPixelDist) {
    return expf(0.0252506f * goalPixelDist) - 1.0f;
}

float vectorMag(float i, float j) {
    return sqrtf((i * i) + (j * j));
}

float vectorPolarAngle(float cartesianAngle) {
    return floatMod(450.0f - cartesianAngle, 360.0f);
}

float vectorI(float mag, float cartesianAngle) {
    float polarAngle = vectorPolarAngle(cartesianAngle);
    return mag * cosf(polarAngle * DEG_TO_RAD_F);
}

float vectorJ(float mag, float cartesianAngle) {
    float polarAngle = vectorPolarAngle(cartesianAngle);
    return mag * sinf(polarAngle * DEG_TO_RAD_F);
}
