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
VoltageDivider voltageDivider;
PID imuPID(IMU_KP, IMU_KI, IMU_KD, IMU_MAX);
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

    voltageDivider.init();
    lightSensors.init();
    camera.init();
    motors.init();

    digitalWrite(LED_BUILTIN, LOW);
}

void getHeading() { // Gets compass heading (relative)
    sensors_event_t imuEvent;
    bno.getEvent(&imuEvent);

    heading = float(imuEvent.orientation.x);

    if (heading > 180.0f) {
        heading -= 360.0f;
    }
}

float getAttackGoalRot() { // Gets attack goal track correction (PID)
    float attackGoalAngle = camera.attackGoalAngle;

    if (attackGoalAngle > 180.0f) {
        attackGoalAngle -= 360.0f;
    }

    float attackGoalRot = -attackGoalTrackPID.update(attackGoalAngle, 0.0f);
    return attackGoalRot;
}

float getDefendGoalRot() { // Gets defend goal track correction (PID)
    float defendGoalAngle = camera.defendGoalAngle;
    float defendGoalRot = -defendGoalTrackPID.update(defendGoalAngle, 180.0f);
    return defendGoalRot;
}

void getOrbitMovement() { // Gets orbit movement
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f); // absolute
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f); // absolute
    float targetAngle = ((camera.attackGoalDist == 0.0f) ? 0.0f : absoluteAttackGoalAngle);
    float dir = floatMod(absoluteBallAngle - targetAngle, 360.0f);

    dir = (dir > 180.0f) ? (dir - 360.0f) : dir; // -180 to 180
    
    float ballAngleDifference = fsign(dir) * fminf(90.0f, 0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, targetAngle))); // Computes angle-based offset
    float strengthFactor = constrain(30.0f / camera.ballDist, 0.0f, 1.0f); // Computes distance scaling factor
    float angleAddition = ballAngleDifference * strengthFactor;

    moveAngle = floatMod((absoluteBallAngle + angleAddition), 360.0f);

    if ((camera.ballDist < 20.0f) && (smallestAngleBetween(absoluteBallAngle, targetAngle) <= 8.0f)) { // surge case
        moveSpeed = 50.0f;
        Serial.println("surge");
    } else {
        moveSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));
        // increase fast speed undershoot more
        // decrease fast speed overshoot more

        // angle addition 90 -> slow speed
        // angle addition 0 -> fast speed

        // increasing fast speed makes robot go back faster
    }
}

void lineAvoid() { // Completely avoid the line
    // moveSpeed = (-8.31025f * (lightSensors.fieldLineSize - 2.0f) * (lightSensors.fieldLineSize - 2.0f)) + 70.0f;
    moveSpeed = -1.0f * expf(-3.1f * lightSensors.fieldLineSize + 3.0f) + 60.0f;
    moveAngle = floatMod(lightSensors.fieldLineAngle + 180.0f, 360.0f);
}

void lineSlide() { // Slide along the line depending on ball position
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

void centerMidField() { // centers in the middle of the field
    float absoluteAttackGoalAngle = floatMod(camera.attackGoalAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);
    float attX = 0.0f, attY = 0.0f, defX = 0.0f, defY = 0.0f;

    if (camera.attackGoalDist != 0.0f) { // attack goal visible
        float polarAttackGoalAngle = floatMod(450.0f - absoluteAttackGoalAngle, 360.0f);
        attX = camera.attackGoalDist * cosf(polarAttackGoalAngle * DEG_TO_RAD_F);
        attY = camera.attackGoalDist * sinf(polarAttackGoalAngle * DEG_TO_RAD_F);
    }

    if (camera.defendGoalDist != 0.0f) { // defend goal visible
        float polarDefendGoalAngle = floatMod(450.0f - absoluteDefendGoalAngle, 360.0f);
        defX = camera.defendGoalDist * cosf(polarDefendGoalAngle * DEG_TO_RAD_F);
        defY = camera.defendGoalDist * sinf(polarDefendGoalAngle * DEG_TO_RAD_F);
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
    Serial.println(sumY);
    float mag = 0.5f * sqrtf((sumX * sumX) + (sumY * sumY)); // magnitude of vector of robot to center of field
    // if (mag <= 10.0f) { // stops jittering
    //     mag = 0.0f;
    // }
    float ang = floatMod(450.0f - (atan2f(sumY, sumX) * RAD_TO_DEG_F), 360.0f); // movement angle
    
    moveSpeed = constrain(1.5f * mag, 0.0f, 40.0f);
    moveAngle = ang;

    // Serial.print(moveSpeed);
    // Serial.print(" ");
    // Serial.print(moveAngle);
    // Serial.println();
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

    rotation = (camera.attackGoalDist == 0.0f) ? (imuPID.update(heading, 0.0f)) : getAttackGoalRot();
}

void defendSearch() { // just moves backwards for now
    moveSpeed = 20.0f;
    moveAngle = 180.0f;
}

void defend() {
    float absoluteBallAngle = floatMod(camera.ballAngle + heading, 360.0f);
    float absoluteDefendGoalAngle = floatMod(camera.defendGoalAngle + heading, 360.0f);

    float fwd = defendVerticalPID.update(camera.defendGoalDist, 30.0f);
    float side = 0.0f;

    if (camera.defendGoalDist != 0.0f) { // goal visible
        if (camera.ballDist != 0.0f) { // ball visible
            if (smallestAngleBetween(absoluteBallAngle, absoluteDefendGoalAngle) < 90.0f) { // ball behind: orbit
                getOrbitMovement();
            } else { // normal defend
                float diff = fsign(absoluteDefendGoalAngle - absoluteBallAngle) * (180.0f - smallestAngleBetween(absoluteDefendGoalAngle, absoluteBallAngle));
                side = defendHorizontalPID.update(diff, 0.0f);
                moveSpeed = sqrtf((side * side) + (fwd * fwd));
                moveAngle = floatMod((450.0f - (atan2f(fwd, side) * RAD_TO_DEG_F)), 360.0f);
            }
        } else { // ball not visible
            // center
            moveSpeed = fabsf(fwd);
            moveAngle = (fwd >= 0.0f) ? floatMod(absoluteDefendGoalAngle + 180.0f, 360.0f) : absoluteDefendGoalAngle;
        }
    } else { // search for defend goal
        defendSearch();
    }

    if (lightSensors.fieldLineSize != -1.0f) { // on line
        if ((lightSensors.fieldLineSize > 0.75f) || (camera.ballDist == 0.0f)) { // line avoid
            lineAvoid();
        } else { // line slide
            lineSlide(); // FIX IT WILL STOP ON LINE WHEN ORBITING
        }        
    } 

    rotation = (camera.defendGoalDist == 0.0f) ? (imuPID.update(heading, 0.0f)) : getDefendGoalRot();
}

void loop() {
    voltageDivider.update();
    getHeading();
    camera.update();
    lightSensors.update(heading);

    #if !COMP
    if (voltageDivider.batteryLow) {
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