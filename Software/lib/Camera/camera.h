#ifndef CAMERA_H
#define CAMERA_H

#include "config.h"
#include "definitions.h"

class Camera {
    public:
        Camera() {}
        
        void init();
        void update(); 

        uint8_t fps = 0;
        
        float ballDist = 0.0f;
        float ballAngle = -1.0f;
        float attackGoalDist = 0.0f;
        float attackGoalAngle = -1.0f;
        float defendGoalDist = 0.0f;
        float defendGoalAngle = -1.0f;

        bool ball = false;
        bool attackGoal = false;
        bool defendGoal = false;

        unsigned long lastTimeBallSeen = millis();
        unsigned long lastTimeAttackGoalSeen = millis();
        unsigned long lastTimeDefendGoalSeen = millis();

        
    private: 
        bool yellowGoal = false;
        bool blueGoal = false;

        float yellowGoalDist = 0.0f;
        float yellowGoalAngle = -1.0f;
        float blueGoalDist = 0.0f;
        float blueGoalAngle = -1.0f;

        uint8_t receivedPacket[12] = {0};
        uint16_t camData[6] = {0};

        float distance(uint16_t cx, uint16_t cy);
        float angle(uint16_t cx, uint16_t cy);

        unsigned long camTime = 0;
        unsigned long lastTimeYellowGoalSeen = millis();
        unsigned long lastTimeBlueGoalSeen = millis();
        
};

#endif