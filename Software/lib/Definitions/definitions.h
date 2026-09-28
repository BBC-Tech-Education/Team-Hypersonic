#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

/* Attack */
#define ATTACK_SLOW_SPEED (ROBOT_1 ? 30.0f : 40.0f)
#define ATTACK_FAST_SPEED (ROBOT_1 ? 30.0f : 40.0f)
#define ATTACK_CLOSE_DISTANCE (ROBOT_1 ? 30.0f : 15.0f) // DON'T CHANGE THIS
#define ATTACK_SURGE_DISTANCE 7.0f
#define ATTACK_SURGE_ANGLE 8.0f
#define ATTACK_SURGE_SPEED 50.0f

/* Defend */
#define DEFEND_SURGE_SPEED 65.0f
#define DEFEND_GOAL_DISTANCE (ROBOT_1 ? 20.0f : 35.0f)
#define DEFEND_SURGE_DISTANCE (ROBOT_1 ? 25.0f : 45.0f)
#define DEFEND_BALL_DISTANCE (ROBOT_1 ? 19.0f : 15.0f)

/* PIDs */
// Defender: lower absoluteMax if overshooting big differences (vertical/horizontal)
// Compass Correct PID
// Increase D, push robot, wait until constant oscillation
// Halve D
// Increase P until constant oscillation
// Halve P
#define IMU_KP (ROBOT_1 ? 0.6f : 0.6f) // not too high (impacts orbit) 
#define IMU_KI (ROBOT_1 ? 0.0f : 0.0f)
#define IMU_KD (ROBOT_1 ? (0.04f / 2.0f) : (0.06f / 2.0f)) // 0.03f
#define IMU_MAX (ROBOT_1 ? 0.0f : 0.0f)

// Attacker Goal Tracking PID
#define ATTACK_GOAL_TRACK_KP (ROBOT_1 ? 0.3f : 0.2f) // high
#define ATTACK_GOAL_TRACK_KI (ROBOT_1 ? 0.0f : 0.0f)
#define ATTACK_GOAL_TRACK_KD (ROBOT_1 ? 0.01f : 0.01f) // no D
#define ATTACK_GOAL_TRACK_MAX (ROBOT_1 ? 30.0f : 30.0f)

// Defender Goal Tracking PID
#define DEFEND_GOAL_TRACK_KP (ROBOT_1 ? 0.3f : 0.5f) // high
#define DEFEND_GOAL_TRACK_KI (ROBOT_1 ? 0.0f : 0.0f)
#define DEFEND_GOAL_TRACK_KD (ROBOT_1 ? 0.0075f : 0.02f) // high D
#define DEFEND_GOAL_TRACK_MAX (ROBOT_1 ? 0.0f : 0.0f)

// Horizontal PID
#define HORIZONTAL_KP (ROBOT_1 ? 1.5f : 1.0f) // low // 0.75, _
#define HORIZONTAL_KI (ROBOT_1 ? 0.0f : 0.0f)
#define HORIZONTAL_KD (ROBOT_1 ? 0.0f : 0.0f) // no D
#define HORIZONTAL_MAX (ROBOT_1 ? 25.0f : 30.0f) // high

// Vertical PID
#define VERTICAL_KP (ROBOT_1 ? 5.0f : 5.0f) // high
#define VERTICAL_KI (ROBOT_1 ? 0.0f : 0.0f)
#define VERTICAL_KD (ROBOT_1 ? 0.0f : 0.0f) // no D
#define VERTICAL_MAX (ROBOT_1 ? 50.0f : 30.0f) // low

/* Pins */
// Voltage Divider
#define VOLTAGE_DIVIDER_PIN 14
#define SCALE_FACTOR (3.3f * 4.3f / 1023.0f) // 0.01387096774f
#define LOW_BATTERY_VOLTAGE 11.1f

// Motors
#define MOTOR_NUM 4
#define MOTOR_PWM_FREQ 15000.0f
#define FL_PWM 28
#define FL_INA 30
#define FL_INB 29
#define FR_PWM 11
#define FR_INA 27
#define FR_INB 12
#define BR_PWM 5
#define BR_INA 7
#define BR_INB 6
#define BL_PWM 8
#define BL_INA 10
#define BL_INB 9

// Light Sensors
#define LS_NUM 48
#define LS_NUM_INNER 32
#define LS_THRESH_BUFF (ROBOT_1 ? 225 : 200)
#define DIMMING (ROBOT_1 ? 125 : 125)
#define LS_0 37
#define LS_1 36
#define LS_2 35
#define LS_3 34
#define LS_OUT_1 40
#define LS_OUT_2 39
#define LS_OUT_3 38
#define LS_PWM 33

// Camera
#define CAMERA_SERIAL Serial1
#define CAMERA_PACKET_NUMBER 14
#define CAMERA_START_BYTE 255
#define CX (ROBOT_1 ? 250 : 240)
#define CY (ROBOT_1 ? 195 : 200)

// Bluetooth
#define BLUETOOTH_SERIAL Serial5
#define BLUETOOTH_PACKET_NUMBER 11
#define BLUETOOTH_START_BYTE 251

// Kicker
#define KICK_DIGITAL 15
#define KICK_ANALOG 41
#define LDR 26
#define KICK_DURATION_MS 3

// Function Declarations
int16_t sign(int16_t value);
float fsign(float value);
float floatMod(float x, float m);
float angleBetween(float angleCounterClockwise, float angleClockwise);
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
int16_t mod(int16_t x, int16_t m);
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck);
float ballPixelToCm(float ballPixelDist);
float goalPixelToCm(float goalPixelDist);
float vectorMag(float i, float j);
float vectorPolarAngle(float cartesianAngle);
float vectorI(float mag, float cartesianAngle);
float vectorJ(float mag, float cartesianAngle);

/* Trigonometry */
#define RAD_TO_DEG_F 57.29578f
#define DEG_TO_RAD_F 0.017453293f

/* FSMs */
enum AttackState {
    ATTACK_ORBIT,
    ATTACK_CENTER,
    ATTACK_PAUSE
};

enum DefendState {
    DEFEND_NORMAL,
    DEFEND_GOAL_CENTER,
    DEFEND_VERTICAL,
    DEFEND_SURGE,
    DEFEND_ORBIT,
    DEFEND_CENTER
};

#define FIELD_LENGTH 200.0f;

#endif
