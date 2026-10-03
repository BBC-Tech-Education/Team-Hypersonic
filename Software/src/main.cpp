#include <Adafruit_BNO055.h>
#include <bluetooth.h>
#include <camera.h>
#include <config.h>
#include <definitions.h>
#include <kicker.h>
#include <lightSensors.h>
#include <motors.h>
#include <PID.h>
#include <voltageDivider.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire2);
Bluetooth bluetooth;
Camera camera;
Kicker kicker;
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

static float posX = 0.0f;
static float posY = 0.0f;

unsigned long lastTimeKicked = millis();
unsigned long kickTimer = millis();
static bool kicking = false;

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
    bluetooth.init();
    // kicker.init();

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

    // rotation = compassCorrectPID.update(heading, 0.0f);
}

void getOrbitMovement() { // Assumes ball is visible DISTANCE MAY BE WRONG
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float targetAngle = camera.attackGoal ? absoluteAttackGoalAngle : 0.0f;
    float direction = floatMod(absoluteBallAngle - targetAngle, 360.0f);
    direction = (direction > 180.0f) ? (direction - 360.0f) : direction;
    
    // #if !ATTACK // check if this works
    // if (camera.defendGoal && (camera.defendGoalDist <= 40.0f)) { // maybe distance is breaking (should be high?)
    //     float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    //     direction = (absoluteDefendGoalAngle <= 180.0f) ? -1.0f : 1.0f;
    // }
    // if (!camera.defendGoal) getAttackRotation();
    // #endif

    if (!camera.defendGoal) getAttackRotation();

    float ballAngleDifference = fsign(direction) * fminf(90.0f, (0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, targetAngle)) - 0.4f));
    // float strengthFactor = constrain(ATTACK_CLOSE_DISTANCE / camera.ballDist, 0.0f, 1.0f);
    float strengthFactor = constrain((1.0f) / (1.0f + expf(0.075f * (camera.ballDist - 40.0f))), 0.0f, 1.0f);
    float angleAddition = ballAngleDifference * strengthFactor;
    moveAngle = floatMod(absoluteBallAngle + angleAddition, 360.0f);
    if ((camera.ballDist < ATTACK_SURGE_DISTANCE) && (smallestAngleBetween(absoluteBallAngle, targetAngle) <= ATTACK_SURGE_ANGLE)) { // check
    // if (camera.ballDist < ATTACK_SURGE_DISTANCE) { // check
        moveAngle = targetAngle;
        moveSpeed = ATTACK_SURGE_SPEED;
    } else {
        moveSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));
        // decreasing fast speed: less overshoot when angle addition is small
        // increasing fast speed: more overshoot when angle addition is small
        // decreasing slow speed: less undershoot when angle addition is large
        // increasing slow speed: more undershoot when angle addition is large
    }
}

void lineAvoid() {
    moveSpeed = expf(0.2f * lightSensors.fieldLineSize + 3.0f) + 75.0f;
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

void getPosition() {
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float attX = 0.0f, attY = 0.0f, defX = 0.0f, defY = 0.0f;

    if (camera.attackGoal) {
        attX = vectorI(camera.attackGoalDist, absoluteAttackGoalAngle);
        attY = vectorJ(camera.attackGoalDist, absoluteAttackGoalAngle);
    }

    if (camera.defendGoal) {
        defX = vectorI(camera.defendGoalDist, absoluteDefendGoalAngle);
        defY = vectorJ(camera.defendGoalDist, absoluteDefendGoalAngle);
    }
        
    if (!camera.attackGoal && !camera.defendGoal) {
        posX = 0.0f;
        posY = 0.0f;
        return;
    }
        
    else if (camera.attackGoal && !camera.defendGoal) {
        defX = attX;
        defY = attY - FIELD_LENGTH;
    }
        
    else if (camera.defendGoal && !camera.attackGoal) {
        attX = defX;
        attY = defY + FIELD_LENGTH;
    }

    posX = -0.5f * (attX + defX);
    posY = -0.5f * (attY + defY);
}

void centerMidField() {
    float sumX = -2.0f * posX;
    float sumY = -2.0f * posY;
    float mag = 0.5f * vectorMag(sumX, sumY); // vector to center of field
    float ang = floatMod(450.0f - (atan2f(sumY, sumX) * RAD_TO_DEG_F), 360.0f); // movement angle
    
    moveSpeed = constrain(1.5f * mag, 0.0f, 45.0f);
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

    moveSpeed = sqrtf((side * side) + (fwd * fwd));
    moveAngle = floatMod((450.0f - (atan2f(fwd, side) * RAD_TO_DEG_F)), 360.0f);
}

void getDefendCentering() {
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float flippedDefendGoalAngle = floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f); // 0 back, increasing clockwise
    float defGoalAngle = (flippedDefendGoalAngle > 180.0f) ? (flippedDefendGoalAngle - 360.0f) : flippedDefendGoalAngle; // back 0, clockwise increases to 180, -180 to 0
    float centerSide = defendHorizontalPID.update(defGoalAngle, 0.0f);
    float fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);

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
            break;

        case DEFEND_SURGE:
            moveSpeed = DEFEND_SURGE_SPEED;
            moveAngle = floatMod(camera.ballAngle + heading, 360.0f);
            break;

        case DEFEND_VERTICAL:
            fwd = defendVerticalPID.update(camera.defendGoalDist, DEFEND_GOAL_DISTANCE);
            moveSpeed = fabsf(fwd);
            moveAngle = (fwd >= 0.0f) ? floatMod(camera.defendGoalAngle + 180.0f + heading, 360.0f) : floatMod(camera.defendGoalAngle + heading, 360.0f);
            break;

        case DEFEND_GOAL_CENTER:
            getDefendCentering();
            break;

        case DEFEND_ORBIT:
            getOrbitMovement();
            break;

        case DEFEND_CENTER:
            centerMidField();
            break;

    }
}

void kick() {
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);

    uint16_t ldrVal = analogRead(LDR);
    uint16_t chargeTime = millis() - lastTimeKicked;
    float kickVoltage = analogRead(KICK_ANALOG) * (3.3f / 1023.0f) * (86.7f / 4.7f); // check scale ratio
    
    bool shouldKick =
        (ldrVal <= 500) && 
        (chargeTime >= 1000) && 
        (kickVoltage >= 47.0f) && 
        camera.ball && 
        (smallestAngleBetween(absoluteBallAngle, heading) <= 5.0f) && 
        (camera.ballDist < ATTACK_SURGE_DISTANCE) && 
        camera.attackGoal && 
        (smallestAngleBetween(absoluteAttackGoalAngle, heading) <= 12.5f) && 
        (camera.attackGoalDist < 60.0f); // tune value
    
    if (shouldKick && !kicking) { // 1st instance
        kicking = true;
        kickTimer = millis();
        digitalWrite(KICK_DIGITAL, LOW);
    }

    if (kicking && (millis() - kickTimer >= KICK_DURATION_MS)) { // 3 right now
        kicking = false;
        lastTimeKicked = millis();
        digitalWrite(KICK_DIGITAL, HIGH);
    }
}

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
        if ((lightSensors.fieldLineSize > 0.09f) || (!camera.ball)) {
            lineAvoid();
        } else {
            lineSlide();
        }   
    } 
    #else
    if (lightSensors.fieldLineSize > 0.09f) {
        lineAvoid();
    } 
    #endif
}

void loop() {
    battery.update();
    getHeading();
    camera.update();
    getPosition();
    lightSensors.update(heading);

    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    // float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    // float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);

    // bluetooth.update(absoluteBallAngle, camera.ballDist, posX, posY);
    // if (bluetooth.attack) attack();
    if (ATTACK) attack();
    else defend();
    updateLine();
    // kick();

    #if !COMP
    if (battery.batteryLow) {
        motors.move(0.0f, 0.0f, 10.0f, 0.0f);
        return;
    } 
    #endif

    #if DEBUG
    debug();
    #endif
    
    // motors.move(0.0f, 0.0f, rotation, heading);
    motors.move(moveSpeed, moveAngle, rotation, heading);
}
