#include "lightSensors.h"

/*
TODO:
-> account for case with 4 inner clusters
*/

void LightSensors::init() { // Initialises light sensors and controls brightness
    pinMode(LS_PWM, OUTPUT);
    analogWrite(LS_PWM, DIMMING);

    for (uint8_t i = 0; i < 3; i++) {
        pinMode(lightOutPins[i], INPUT);
    }

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(lightPins[i], OUTPUT);
    }

    delay(100);

    read();

    for (uint8_t i = 0; i < LS_NUM; i++) {
        whiteCal[i] = lightValues[i] + LS_THRESH_BUFF;
    }
}

void LightSensors::read() { // Reads all phototransistors
    for (uint8_t i = 0; i < 16; i++) {
        for (uint8_t j = 0; j < 4; j++) { // Loop through all 16 mux address combos (0000 -> 1111)
            // Set the 4 multiplexer address pins according to the bits of i
            // Example: i = 5 (0101) → pins = HIGH, LOW, HIGH, LOW
            digitalWrite(lightPins[j], (i >> j) & 0x01);
        }

        delayMicroseconds(10); // allows voltage to settle

        lightValues[muxIndex[i]]      = analogRead(LS_OUT_1);
        lightValues[muxIndex[i + 16]] = analogRead(LS_OUT_2);
        lightValues[muxIndex[i + 32]] = analogRead(LS_OUT_3);
    }
}

void LightSensors::update(float heading) {
    read();

    for (uint8_t i = 0; i < LS_NUM; i++) { // finds which sensors are on the line
        whiteValues[i] = (lightValues[i] > whiteCal[i]);
    }

    for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
        if (!whiteValues[i]) { // if a sensor is not white, check if its neighbours are white
            whiteValues[i] = (whiteValues[mod(i - 1, LS_NUM_INNER)] && whiteValues[mod(i + 1, LS_NUM_INNER)]);
        }
    }

    // for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
    //     Serial.print(whiteValues[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    clusterNum = 0;
    inCluster = false;
    LightSensors::Cluster ClusterArray[4];
    lineDir = -1.0f;
    lineSize = -1.0f;

    // inner ring

    for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
        if (!inCluster) {
            if (whiteValues[i]) { // start cluster
                ClusterArray[clusterNum].start = i;
                inCluster = true;
            }
        } else {
            if (!whiteValues[i]) { // end cluster
                ClusterArray[clusterNum].end = i - 1;
                inCluster = false;
                clusterNum++;
            }
        }
    }
    
    if (whiteValues[LS_NUM_INNER - 1]) { // will the fill in dead sensors affect this
        if (whiteValues[0]) { // merges first and last clusters
            ClusterArray[0].start = ClusterArray[clusterNum].start;
        } else {
            ClusterArray[clusterNum].end = LS_NUM_INNER - 1;
            clusterNum++;
        }
    }

    if (clusterNum > 0) {
        for (uint8_t i = 0; i < clusterNum; i++) {
            ClusterArray[i].midpoint = midAngleBetween(ClusterArray[i].start * 11.25f, ClusterArray[i].end * 11.25f);
        }

        if (clusterNum == 3) {
            // line angle is the midpoint of the two clusters with the largest angle between them
            float angleDiff01 = angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
            float angleDiff12 = angleBetween(ClusterArray[1].midpoint, ClusterArray[2].midpoint);
            float angleDiff20 = angleBetween(ClusterArray[2].midpoint, ClusterArray[0].midpoint);
            // adds up to 360
            float biggestAngle = fmaxf(angleDiff01, fmaxf(angleDiff12, angleDiff20));

            if (biggestAngle == angleDiff01) { // Line size calculated from how far robot is over line
                lineDir = midAngleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint);
                lineSize = angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) / 2.0f) : 1.0f;
            } else if (biggestAngle == angleDiff12) {
                lineDir = midAngleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint);
                lineSize = angleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint) / 2.0f) : 1.0f;
            } else {
                lineDir = midAngleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint);
                lineSize = angleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint) / 2.0f) : 1.0f;
            }
        } else if (clusterNum == 2) { // line angle midpoint of minor arc between the two clusters
            bool clockwise = angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) <= 180.0f;
            lineDir = clockwise ? midAngleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) : midAngleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint);
            lineSize = 1.0f - cosf(DEG_TO_RAD_F * (clockwise ? angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) / 2.0f : angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) / 2.0f));
        } else { // line angle midpoint of singular cluster
            lineDir = ClusterArray[0].midpoint;
            lineSize = 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[0].start * 11.25f, ClusterArray[0].end * 11.25f) / 2.0f);
        }
    } else { // check outer ring
        if (whiteValues[32] || whiteValues[33] || whiteValues[34] || whiteValues[35]) { // front
            ClusterArray[0].midpoint = 0.0f;
            clusterNum ++;
        }

        if (whiteValues[36] || whiteValues[37] || whiteValues[38] || whiteValues[39]) { // right
            ClusterArray[1].midpoint = 90.0f;
            clusterNum ++;
        }

        if (whiteValues[40] || whiteValues[41] || whiteValues[42] || whiteValues[43]) { // back
            ClusterArray[2].midpoint = 180.0f;
            clusterNum ++;
        }

        if (whiteValues[44] || whiteValues[45] || whiteValues[46] || whiteValues[47]) { // left
            ClusterArray[3].midpoint = 270.0f;
            clusterNum ++;
        }

        if (clusterNum == 2) {
            if (ClusterArray[0].midpoint != -1.0f) {
                if (ClusterArray[1].midpoint != -1.0f) { // front and right
                    lineDir = 45.0f;
                    lineSize = 0.1f;
                } else if (ClusterArray[3].midpoint != -1.0f) { // front and left
                    lineDir = 315.0f;
                    lineSize = 0.1f;
                }
            } else if (ClusterArray[2].midpoint != -1.0f) {
                if (ClusterArray[1].midpoint != -1.0f) { // right and back
                    lineDir = 135.0f;
                    lineSize = 0.1f;
                } else if (ClusterArray[3].midpoint != -1.0f) { // back and left
                    lineDir = 225.0f;
                    lineSize = 0.1f;
                }
            }
        } else if (clusterNum == 1) {
            for (uint8_t i = 0; i < 4; i++) {
                if (ClusterArray[i].midpoint != -1.0f) { // just the midpoint
                    lineDir = ClusterArray[i].midpoint;
                }
            }
            lineSize = 0.1f; // small
        }

    }

    // Serial.printf("Direction: %.2f\tSize: %.2f\n", lineDir, lineSize);

    bool noLine = (lineDir == -1.0f);
    float lineDirection = noLine ? -1.0f : floatMod(450.0f - lineDir + heading, 360.0f); // absolute

    if (onField) {
        if (lineDir != -1.0f) { // in field but on line
            fieldLineAngle = lineDirection;
            fieldLineSize = lineSize;
            onField = false;
        }
    } else {
        if (fieldLineSize == 3.0f) { // outside field
            if (lineDir != -1.0f) {  // on other side of line (flip line angle)
                // ignores case where robot goes into goal box and sees goal line
                // if ((lineDir != -1.0f) && (smallestAngleBetween(lineDirection, fieldLineAngle) >= 60.0f)) { 
                fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                fieldLineSize = 2.0f - lineSize;
            }
        } else {  
            if (lineDir == -1.0f) { // no line
                if (fieldLineSize <= 1.0f) { // fully in field
                    onField = true;
                    fieldLineAngle = -1.0f;
                    fieldLineSize = -1.0f;
                } else { // fully outside field
                    fieldLineSize = 3.0f;
                }
            } else {
                if (smallestAngleBetween(lineDirection, fieldLineAngle) <= 90.0f) { // inside field but on line
                    fieldLineAngle = lineDirection;
                    fieldLineSize = lineSize;
                } else { // outside field but on line (flip line angle)
                    fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                    fieldLineSize = 2.0f - lineSize;
                }
            }
        }
    }

    // Serial.print("\t");
    // Serial.print(lineSize);
    // Serial.print(" ");
    // Serial.print(fieldLineSize);
    // Serial.print(" ");
    // Serial.print(lineDir);
    // Serial.print(" ");
    // Serial.print(fieldLineAngle);
    // Serial.print(" ");
    // Serial.print(onField);
    // Serial.print(" ");
    // Serial.print(clusterNum);
    // Serial.println();
}