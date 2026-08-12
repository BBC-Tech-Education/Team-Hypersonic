#include <Adafruit_BNO055.h>
#include <camera.h>
#include <config.h>
#include <definitions.h>
#include <lightSensors.h>
#include <motors.h>
#include <PID.h>
#include <voltageDivider.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire2);
Camera camera;
LightSensors lightSensors;
Motors motors;
VoltageDivider battery;
PID compassCorrectPID(IMU_KP, IMU_KI, IMU_KD, IMU_MAX);
PID attackGoalTrackPID(ATTACK_GOAL_TRACK_KP, ATTACK_GOAL_TRACK_KI, ATTACK_GOAL_TRACK_KD, ATTACK_GOAL_TRACK_MAX);
PID defendGoalTrackPID(DEFEND_GOAL_TRACK_KP, DEFEND_GOAL_TRACK_KI, DEFEND_GOAL_TRACK_KD, DEFEND_GOAL_TRACK_MAX);
PID defendHorizontalPID(HORIZONTAL_KP, HORIZONTAL_KI, HORIZONTAL_KD, HORIZONTAL_MAX);
PID defendVerticalPID(VERTICAL_KP, VERTICAL_KI, VERTICAL_KD, VERTICAL_MAX);

static float moveSpeed = 0.0f;
static float moveAngle = 0.0f;
static float rotation = 0.0f;
static float heading = 0.0f;

unsigned long lastTimeBallSeen = millis();

void setup() {
    delay(100);
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    while (!bno.begin(OPERATION_MODE_IMUPLUS)) { 
        Serial.println("BNO not working");
        delay(1000);
    }

    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);

    battery.init();
    lightSensors.init();
    camera.init();
    motors.init();

    digitalWrite(LED_BUILTIN, LOW);
}

void getHeading() {
    sensors_event_t imuEvent;
    bno.getEvent(&imuEvent);
    heading = float(imuEvent.orientation.x);

    if (heading > 180.0f) {
        heading -= 360.0f;
    }
}

float getAttackGoalRot() {
    float attackGoalAngle = camera.attackGoalAngle;

    if (attackGoalAngle > 180.0f) {
        attackGoalAngle -= 360.0f;
    }

    float attackGoalRotation = -1.0f * attackGoalTrackPID.update(attackGoalAngle, 0.0f);
    return attackGoalRotation;
}

float getDefendGoalRot() {
    float defendGoalAngle = camera.defendGoalAngle;
    float defendGoalRotation = -defendGoalTrackPID.update(defendGoalAngle, 180.0f);
    return defendGoalRotation;
}

void getOrbitMovement() {
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float targetAngle = ((camera.attackGoalDist == 0.0f) ? 0.0f : absoluteAttackGoalAngle);
    float direction = floatMod(absoluteBallAngle - targetAngle, 360.0f);
    direction = (direction > 180.0f) ? (direction - 360.0f) : direction; // -180 to 180
    
    #if !ATTACK
    if ((camera.defendGoalDist != 0.0f) && (camera.defendGoalDist <= 40.0f)) {
        float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
        if (absoluteDefendGoalAngle <= 180.0f) { // robot on left side of goal
            direction = -1.0f;
        } else { // robot on right side of goal
            direction = 1.0f;
        }
    }
    #endif

    float ballAngleDifference = fsign(direction) * fminf(90.0f, 0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, targetAngle)));
    float strengthFactor = constrain(ATTACK_CLOSE_DISTANCE / camera.ballDist, 0.0f, 1.0f);
    float angleAddition = ballAngleDifference * strengthFactor;

    moveAngle = floatMod(absoluteBallAngle + angleAddition, 360.0f);

    if ((camera.ballDist < ATTACK_SURGE_DISTANCE) && (smallestAngleBetween(absoluteBallAngle, targetAngle) <= ATTACK_SURGE_ANGLE)) { // surge
        moveSpeed = ATTACK_SURGE_SPEED;
    } else {
        moveSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));
        // angle addition 90 -> slow speed
        // angle addition 0 -> fast speed
        // decreasing fast speed: less overshoot when angle addition is small
        // increasing fast speed: more overshoot when angle addition is small
        // decreasing slow speed: less undershoot when angle addition is large
        // increasing slow speed: more undershoot when angle addition is large
    }
}

void lineAvoid() {
    moveSpeed = -1.0f * expf(-3.1f * lightSensors.fieldLineSize + 3.0f) + 60.0f;
    moveAngle = floatMod(lightSensors.fieldLineAngle + 180.0f, 360.0f);
}

void lineSlide() {
    float diff = floatMod(moveAngle - lightSensors.fieldLineAngle, 360.0f);
    diff = (diff > 180.0f) ? (diff - 360.0f) : diff; // -180 to 180

    if (fabsf(diff) < 90.0f) { // ball is outside field
        if (diff > 0.0f) {
            moveAngle = floatMod(lightSensors.fieldLineAngle + 90.0f, 360.0f);
        } else {
            moveAngle = floatMod(lightSensors.fieldLineAngle - 90.0f, 360.0f);
        }
        moveSpeed = fabsf(sinf(diff * DEG_TO_RAD_F)) * 40.0f;
    }
}

void centerMidField() {
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float attX = 0.0f, attY = 0.0f, defX = 0.0f, defY = 0.0f;

    if (camera.attackGoalDist != 0.0f) { // attack goal visible
        attX = vectorI(camera.attackGoalDist, absoluteAttackGoalAngle);
        attY = vectorJ(camera.attackGoalDist, absoluteAttackGoalAngle);
    }

    if (camera.defendGoalDist != 0.0f) { // defend goal visible
        defX = vectorI(camera.defendGoalDist, absoluteDefendGoalAngle);
        defY = vectorJ(camera.defendGoalDist, absoluteDefendGoalAngle);
    }

    if ((camera.attackGoalDist == 0.0f) && (camera.defendGoalDist == 0.0f)) { // both goals not visible
        moveSpeed = 0.0f;
        moveAngle = -1.0f;
        return;
    } else if ((camera.attackGoalDist != 0.0f) && (camera.defendGoalDist == 0.0f)) { // attack goal visible only
        defX = attX;
        defY = attY - 200.0f;
    } else if ((camera.defendGoalDist != 0.0f) && (camera.attackGoalDist == 0.0f)) { // defend goal visible only
        attX = defX;
        attY = defY + 200.0f;
    }

    float sumX = attX + defX;
    float sumY = attY + defY;
    float mag = 0.5f * vectorMag(sumX, sumY); // vector to center of field

    // if (mag <= 10.0f) { // stops jittering
    //     mag = 0.0f;
    // }

    float ang = floatMod(450.0f - (atan2f(sumY, sumX) * RAD_TO_DEG_F), 360.0f); // movement angle
    
    moveSpeed = constrain(1.5f * mag, 0.0f, 40.0f);
    moveAngle = ang;
}

void attack() {
    if (camera.ballDist != 0.0f) { // ball visible
        lastTimeBallSeen = millis();
        getOrbitMovement(); // orbit
    } else { // ball not visible
        if ((millis() - lastTimeBallSeen) > 1000) { // more than 1s since ball last seen
            centerMidField();
        } else { // stop momentarily
            moveSpeed = 0.0f;
            moveAngle = -1.0f;
        }
    }

    if (lightSensors.fieldLineSize != -1.0f) { // on line
        if ((lightSensors.fieldLineSize > 0.75f) || (camera.ballDist == 0.0f)) { // line avoid
            lineAvoid();
        } else { // line slide
            lineSlide();
        }        
    } 

    rotation = (camera.attackGoalDist == 0.0f) ? (compassCorrectPID.update(heading, 0.0f)) : getAttackGoalRot();
}

void defend() { // FIX ORBIT OSCILLATION WHEN GOAL ANGLE ~ 180
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float direction = floatMod(absoluteDefendGoalAngle - absoluteBallAngle, 360.0f);
    direction = (direction > 180.0f) ? (direction - 360.0f) : direction; // -180 to 180
    float ballDiff = fsign(direction) * (smallestAngleBetween(floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f), absoluteBallAngle));
    float fwd = defendVerticalPID.update(camera.defendGoalDist, 30.0f);
    float side = 0.0f;

    uint8_t defendState;
    
    if (camera.defendGoalDist != 0.0f) {
        defendState = (camera.ballDist != 0.0f) ? 1 : 2;
    } else {
        defendState = (camera.ballDist != 0.0f) ? 3 : 4;
    }

    switch (defendState) { // change orbit to if goal on left orbit left if goal on right orbit right
        case 1: // Both goal and ball visible
            lastTimeBallSeen = millis();
            if (fabsf(ballDiff) >= 90.0f) { // ball behind: orbit
                getOrbitMovement();
            } else { // normal defend
                if ((camera.defendGoalDist <= 40.0f) && (camera.ballDist <= 30.0f)) { // surge case
                    moveSpeed = 50.0f;
                    moveAngle = absoluteBallAngle;
                } else {
                    side = -1.0f * defendHorizontalPID.update(ballDiff, 0.0f);
                    moveSpeed = sqrtf((side * side) + (fwd * fwd));
                    moveAngle = floatMod((450.0f - (atan2f(fwd, side) * RAD_TO_DEG_F)), 360.0f);
                }
            }
            break;
        case 2: // Goal visible but ball not visible
            if ((millis() - lastTimeBallSeen) > 1000) { // center
                float flippedDefendGoalAngle = floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f); // 0 back, increasing clockwise
                float defGoalAngle = (flippedDefendGoalAngle > 180.0f) ? (flippedDefendGoalAngle - 360.0f) : flippedDefendGoalAngle; // back 0, clockwise increases to 180, -180 to 0
                float centerSide = defendHorizontalPID.update(defGoalAngle, 0.0f); // may be negative
                moveSpeed = sqrtf((centerSide * centerSide) + (fwd * fwd));
                moveAngle = floatMod((450.0f - (atan2f(fwd, centerSide) * RAD_TO_DEG_F)), 360.0f);
            } else {
                moveSpeed = fabsf(fwd);
                moveAngle = (fwd >= 0.0f) ? floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f) : absoluteDefendGoalAngle;
            }
            break;
        case 3: // Goal not visible but ball visible
            lastTimeBallSeen = millis();
            getOrbitMovement();
            break;
        case 4: // None visible
            if (millis() - lastTimeBallSeen > 1000) {
                centerMidField();
            } else {
                moveSpeed = 0.0f;
                moveAngle = -1.0f;
            }
            break;
    }

    if (lightSensors.fieldLineSize != -1.0f) { // on line
        if ((lightSensors.fieldLineSize > 0.75f) || (camera.ballDist == 0.0f)) { // line avoid
            lineAvoid();
        } else { // line slide
            lineSlide();
        }   
    } 

    rotation = (camera.defendGoalDist == 0.0f) ? (compassCorrectPID.update(heading, 0.0f)) : getDefendGoalRot();
    
    if (defendState == 3) { // f it we attacking
        rotation = (camera.attackGoalDist == 0.0f) ? (compassCorrectPID.update(heading, 0.0f)) : getAttackGoalRot();
    }
}

void loop() {
    battery.update();
    getHeading();
    camera.update();
    lightSensors.update(heading);

    #if !COMP
    if (battery.batteryLow) {
        motors.move(0.0f, 0.0f, 10.0f, 0.0f);
        return;
    } 
    #endif  
    
    #if ATTACK
    attack();
    #else
    defend();
    #endif
    
    motors.move(moveSpeed, moveAngle, rotation, heading);
}
