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

/* 
TODO:
-> Check defender orbit works properly
-> Check orbit surge distances and angles
-> Check defend surge distances and angles
-> Add superteam line searching algorithm into centerMidField
*/

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
    motors.init();
    camera.init();
    lightSensors.init();

    digitalWrite(LED_BUILTIN, LOW);
}

void getHeading() {
    sensors_event_t imuEvent;
    bno.getEvent(&imuEvent);
    heading = float(imuEvent.orientation.x);
    if (heading > 180.0f) heading -= 360.0f;
}

void getAttackRotation() {
    float attackGoalAngle = camera.attackGoalAngle;
    if (attackGoalAngle > 180.0f) attackGoalAngle -= 360.0f;
    float attackGoalRotation = -1.0f * attackGoalTrackPID.update(attackGoalAngle, 0.0f);
    
    rotation = camera.attackGoal
    ? attackGoalRotation
    : compassCorrectPID.update(heading, 0.0f);
    // rotation = compassCorrectPID.update(heading, 0.0f);
}

void getDefendRotation() {
    float defendGoalRotation = -1.0f * defendGoalTrackPID.update(camera.defendGoalAngle, 180.0f);

    rotation = camera.defendGoal
    ? defendGoalRotation 
    : compassCorrectPID.update(heading, 0.0f);
}

void getOrbitMovement() { // Assumes ball is visible DISTANCE MAY BE WRONG
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float targetAngle = camera.attackGoal ? absoluteAttackGoalAngle : 0.0f;
    float direction = floatMod(absoluteBallAngle - targetAngle, 360.0f);
    direction = (direction > 180.0f) ? (direction - 360.0f) : direction;
    
    #if !ATTACK // check if this works
    if (camera.defendGoal && (camera.defendGoalDist <= 40.0f)) { // maybe distance is breaking (should be high?)
        float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
        direction = (absoluteDefendGoalAngle <= 180.0f) ? -1.0f : 1.0f;
    }
    if (!camera.defendGoal) getAttackRotation();
    #endif

    float ballAngleDifference = fsign(direction) * fminf(90.0f, (0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, targetAngle)) - 0.4f));
    float strengthFactor = constrain(ATTACK_CLOSE_DISTANCE / camera.ballDist, 0.0f, 1.0f);
    float angleAddition = ballAngleDifference * strengthFactor;

    moveAngle = floatMod(absoluteBallAngle + angleAddition, 360.0f);

    if ((camera.ballDist < ATTACK_SURGE_DISTANCE) && (smallestAngleBetween(absoluteBallAngle, targetAngle) <= ATTACK_SURGE_ANGLE)) { // check
        moveSpeed = ATTACK_SURGE_SPEED;
    } else {
        moveSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));
        // decreasing fast speed: less overshoot when angle addition is small
        // increasing fast speed: more overshoot when angle addition is small
        // decreasing slow speed: less undershoot when angle addition is large
        // increasing slow speed: more undershoot when angle addition is large
    }
    // Serial.println(angleAddition);
}

void lineAvoid() {
    moveSpeed = -1.0f * expf(-3.1f * lightSensors.fieldLineSize + 3.0f) + 75.0f;
    moveAngle = floatMod(lightSensors.fieldLineAngle + 180.0f, 360.0f);
}

void lineSlide() {
    float diff = floatMod(moveAngle - lightSensors.fieldLineAngle, 360.0f);
    diff = (diff > 180.0f) ? (diff - 360.0f) : diff;

    if (fabsf(diff) < 90.0f) { // moveAngle goes outside field
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

    if (camera.attackGoal) { // attack goal visible
        attX = vectorI(camera.attackGoalDist, absoluteAttackGoalAngle);
        attY = vectorJ(camera.attackGoalDist, absoluteAttackGoalAngle);
    }

    if (camera.defendGoal) { // defend goal visible
        defX = vectorI(camera.defendGoalDist, absoluteDefendGoalAngle);
        defY = vectorJ(camera.defendGoalDist, absoluteDefendGoalAngle);
    }

    if ((!camera.attackGoal) && (!camera.defendGoal)) { // both goals not visible
        moveSpeed = 0.0f;
        moveAngle = -1.0f; // change to superteam searching algorithm
        return;
    } else if ((camera.attackGoal) && (!camera.defendGoal)) { // attack goal visible only
        defX = attX;
        defY = attY - FIELD_LENGTH;
    } else if ((camera.defendGoal) && (!camera.attackGoal)) { // defend goal visible only
        attX = defX;
        attY = defY + FIELD_LENGTH;
    }

    float sumX = attX + defX;
    float sumY = attY + defY;
    float mag = 0.5f * vectorMag(sumX, sumY); // vector to center of field
    float ang = floatMod(450.0f - (atan2f(sumY, sumX) * RAD_TO_DEG_F), 360.0f); // movement angle
    
    moveSpeed = constrain(1.5f * mag, 0.0f, 40.0f);
    moveAngle = ang;
}

AttackState getAttackState() {
    if (camera.ball) return ATTACK_ORBIT;

    if ((millis() - camera.lastTimeBallSeen) > 1000) return ATTACK_CENTER;

    return ATTACK_PAUSE;
}

void attack() {
    AttackState attackState = getAttackState();
    getAttackRotation();

    switch (attackState) {
        case ATTACK_ORBIT:
            getOrbitMovement();
            break;
        
        case ATTACK_PAUSE:
            moveSpeed = 0.0f;
            moveAngle = -1.0f;
            break;

        case ATTACK_CENTER:
            centerMidField();
            break;

    }
}

DefendState getDefendState() {
    if (camera.ball && camera.defendGoal) { // Ball and defend goal visible
        if (smallestAngleBetween(camera.ballAngle, camera.defendGoalAngle) <= 90.0f) return DEFEND_ORBIT;
        if ((camera.defendGoalDist <= DEFEND_SURGE_DISTANCE) && (camera.ballDist <= DEFEND_BALL_DISTANCE)) return DEFEND_SURGE; // check
        return DEFEND_NORMAL;
    }

    else if (!camera.ball && camera.defendGoal) { // Only defend goal visible
        if ((millis() - camera.lastTimeBallSeen) > 1000) return DEFEND_GOAL_CENTER;
        return DEFEND_VERTICAL;
    } 

    else if (camera.ball && !camera.defendGoal) return DEFEND_ORBIT; // Only ball visible

    return DEFEND_CENTER;
}

void getDefendMovement() {
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float direction = floatMod(absoluteDefendGoalAngle - absoluteBallAngle, 360.0f);
    direction = (direction > 180.0f) ? (direction - 360.0f) : direction; // -180 to 180
    float ballDiff = fsign(direction) * (smallestAngleBetween(floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f), absoluteBallAngle));
    float side = -1.0f * defendHorizontalPID.update(ballDiff, 0.0f);
    float fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);
    // Serial.print(side);
    // Serial.print(" ");
    // Serial.print(fwd);
    // Serial.println();

    moveSpeed = sqrtf((side * side) + (fwd * fwd));
    moveAngle = floatMod((450.0f - (atan2f(fwd, side) * RAD_TO_DEG_F)), 360.0f);
}

void getDefendCentering() {
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float flippedDefendGoalAngle = floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f); // 0 back, increasing clockwise
    float defGoalAngle = (flippedDefendGoalAngle > 180.0f) ? (flippedDefendGoalAngle - 360.0f) : flippedDefendGoalAngle; // back 0, clockwise increases to 180, -180 to 0
    float centerSide = defendHorizontalPID.update(defGoalAngle, 0.0f);
    float fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);

    // Serial.print(centerSide);
    // Serial.print(" ");
    // Serial.print(fwd);
    // Serial.println();

    moveSpeed = sqrtf((centerSide * centerSide) + (fwd * fwd));
    moveAngle = floatMod((450.0f - (atan2f(fwd, centerSide) * RAD_TO_DEG_F)), 360.0f);
}

void defend() {
    DefendState defendState = getDefendState();
    getDefendRotation();

    float fwd = 0.0f;

    switch (defendState) {
        case DEFEND_NORMAL:
            getDefendMovement();
            // Serial.println("D1");
            break;

        case DEFEND_SURGE:
            moveSpeed = DEFEND_SURGE_SPEED;
            moveAngle = floatMod(camera.ballAngle + heading, 360.0f);
            // Serial.println("D2");
            break;

        case DEFEND_VERTICAL:
            fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);
            moveSpeed = fabsf(fwd);
            moveAngle = (fwd >= 0.0f) ? floatMod(camera.defendGoalAngle + 180.0f + heading, 360.0f) : floatMod(camera.defendGoalAngle + heading, 360.0f);
            // Serial.println("D3");
            break;

        case DEFEND_GOAL_CENTER:
            getDefendCentering();
            // Serial.println("D4");
            break;

        case DEFEND_ORBIT:
            getOrbitMovement();
            // Serial.println("D5");
            break;

        case DEFEND_CENTER:
            centerMidField();
            // Serial.println("D6");
            break;

    }
    // Serial.println(camera.defendGoalDist);
}

// void oldDefend() {
//     float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
//     float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
//     float direction = floatMod(absoluteDefendGoalAngle - absoluteBallAngle, 360.0f);
//     direction = (direction > 180.0f) ? (direction - 360.0f) : direction; // -180 to 180
//     float fwd = 0.0f;
//     float ballDiff = 0.0f;
//     float side = 0.0f;

//     uint8_t defendState;
    
//     if (camera.defendGoal) {
//         defendState = (camera.ballDist != 0.0f) ? 1 : 2;
//     } else {
//         defendState = (camera.ballDist != 0.0f) ? 3 : 4;
//     }

//     switch (defendState) { // change orbit to if goal on left orbit left if goal on right orbit right
//         case 1: // Both goal and ball visible
//             ballDiff = fsign(direction) * (smallestAngleBetween(floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f), absoluteBallAngle));
//             if (smallestAngleBetween(absoluteBallAngle, absoluteDefendGoalAngle) <= 90.0f) { // ball behind: orbit
//                 getOrbitMovement();
//             } else { // normal defend
//                 if ((camera.defendGoalDist <= DEFEND_GOAL_DISTANCE) && (camera.ballDist <= DEFEND_BALL_DISTANCE)) { // surge case
//                     moveSpeed = 50.0f;
//                     moveAngle = absoluteBallAngle;
//                 } else {
//                     side = -1.0f * defendHorizontalPID.update(ballDiff, 0.0f);
//                     fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);
//                     moveSpeed = sqrtf((side * side) + (fwd * fwd));
//                     moveAngle = floatMod((450.0f - (atan2f(fwd, side) * RAD_TO_DEG_F)), 360.0f);
//                 }
//             }
//             break;
//         case 2: // Goal visible but ball not visible
//             fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);
//             if ((millis() - camera.lastTimeBallSeen) > 1000) { // center
//                 float flippedDefendGoalAngle = floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f); // 0 back, increasing clockwise
//                 float defGoalAngle = (flippedDefendGoalAngle > 180.0f) ? (flippedDefendGoalAngle - 360.0f) : flippedDefendGoalAngle; // back 0, clockwise increases to 180, -180 to 0
//                 float centerSide = defendHorizontalPID.update(defGoalAngle, 0.0f); // may be negative
//                 moveSpeed = sqrtf((centerSide * centerSide) + (fwd * fwd));
//                 moveAngle = floatMod((450.0f - (atan2f(fwd, centerSide) * RAD_TO_DEG_F)), 360.0f);
//             } else {
//                 moveSpeed = fabsf(fwd);
//                 moveAngle = (fwd >= 0.0f) ? floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f) : absoluteDefendGoalAngle;
//             }
//             break;
//         case 3: // Goal not visible but ball visible
//             getOrbitMovement();
//             // Serial.println("O");
//             break;
//         case 4: // None visible
//             if (millis() - camera.lastTimeBallSeen > 1000) {
//                 centerMidField();
//                 // Serial.println("CMF");
//             } else {
//                 moveSpeed = 0.0f;
//                 moveAngle = -1.0f;
//                 // Serial.println("IDK");
//             }
//             break;
//     }
//     // Serial.println(fwd);
// }

void debug() {
    #if DEBUG_BATTERY
    Serial.printf("Battery Voltage: %.2f", battery.battVoltage);
    #endif

    #if DEBUG_IMU
    Serial.printf("Heading: %.2f Compass Correct: %.2f Mode: %d", heading, getCompassRotation(), (uint8_t)bno.getMode());
    #endif

    #if DEBUG_CAMERA
    Serial.printf("bAng: %.2f bDist: %.2f attGoalAng: %.2f attGoalDist: %.2f defGoalAng: %.2f defGoalDist: %.2f FPS: %d", camera.ballAngle, camera.ballDist, camera.attackGoalAngle, camera.attackGoalDist, camera.defendGoalAngle, camera.defendGoalDist, camera.fps);
    #endif

    #if DEBUG_LS
    Serial.printf("fieldLineAngle: %.2f\tfieldLineSize: %.2f", lightSensors.fieldLineAngle, lightSensors.fieldLineSize);
    #endif

    #if DEBUG
    Serial.println();
    #endif
}

void updateLine() {
    #if ATTACK
    if (lightSensors.fieldLineSize != -1.0f) {
        if ((lightSensors.fieldLineSize > 0.75f) || (camera.ballDist == 0.0f)) {
            lineAvoid();
        } else { // line slide
            lineSlide();
        }   
    } 
    #else
    if (lightSensors.fieldLineSize > 0.35f) {
        lineAvoid();
    } 
    #endif
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

    updateLine();
    
    #if DEBUG
    debug();
    #endif

    motors.move(moveSpeed, moveAngle, rotation, heading);
    // motors.move(0.0f, 0.0f, rotation, heading);
}
