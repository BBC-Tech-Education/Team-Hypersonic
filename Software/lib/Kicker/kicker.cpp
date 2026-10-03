#include "kicker.h"

void Kicker::init() {
    pinMode(KICK_DIGITAL, OUTPUT);
    pinMode(KICK_ANALOG, INPUT);
    // pinMode(LDR, INPUT); // MAKE SURE YOU DON'T KILL THE TEENSY PIN
    digitalWrite(KICK_DIGITAL, LOW); // default high?
}

// void Kicker::update(float heading, bool ball, float ballAngle, float ballDist, bool attackGoal, float attackGoalDist) {
//     ldrVal = analogRead(LDR);

//     uint16_t chargeTimer = millis() - lastTimeKicked;
//     float kickVoltage = analogRead(KICK_ANALOG) * (3.3f * 4.3f / 1023.0f); // fix scale ratio

//     if (
//         (ldrVal <= 500)                                       // laser covered by ball
//         && (chargeTimer >= 1000)                              // more than 1s since last kicked
//         && (kickVoltage >= 47.0f)                             // voltage high enough
//         && ball                                               // ball visible
//         && (smallestAngleBetween(ballAngle, heading) <= 5.0f) // ball in capture zone
//         && (ballDist < ATTACK_SURGE_DISTANCE)                 // ball close
//         && attackGoal

//     ) { // kick
//         // 1 second since last kick
//         kick = true;
//     }

//     if (kick) {
//         /// 
//         lastTimeKicked = millis();
//     } else {
//         kick = false;
//     }
    
// }