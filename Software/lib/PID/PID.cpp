#include "PID.h"

// Proportional: Multiplies error by kp. Speeds up response but can cause oscillations
// Integral: Sums error over time and multiplies it by ki. Excessive gain makes controller unstable
// Derivative: Multiplies rate of change of error by kd. Stops overshooting

PID::PID(float p, float i, float d, float absoluteMax) {
    kp = p;
    ki = i;
    kd = d;
    absMax = absoluteMax;
    lastTime = micros();
}

// float PID::update(float input, float setpoint, float modulus) {
//     float derivative;
//     float error = setpoint - input;
//     unsigned long currentTime = micros();
//     float elapsedTime = (currentTime - lastTime) / 1000000.0f;
//     lastTime = currentTime;
//     integral += elapsedTime * error;
//     if (modulus != 0.0f) {
//         float difference = (error - lastError);
//         if (difference < -modulus) {
//             difference += modulus;
//         } else if (difference > modulus) {
//             difference -= modulus;
//         }
//         derivative = difference / elapsedTime;
//     } else {
//         derivative = -(error - lastError) / elapsedTime;
//     }
//     lastError = error;
//     float correction = kp * error + ki * integral - kd * derivative;
//     return absMax == 0.0f ? correction : constrain(correction, -absMax, absMax);
// }

float PID::update(float input, float setpoint, float modulus) {
   unsigned long currentTime = micros();
   float elapsedTime = (currentTime - lastTime) / 1000000.0f;
   lastTime = currentTime;

   if (!isfinite(input) || !isfinite(setpoint)) return 0.0f;
   if (elapsedTime <= 0.0f) elapsedTime = 1e-6f;

   float error = setpoint - input;

   integral += elapsedTime * error;
   if (!isfinite(integral)) integral = 0.0f;
   integral = constrain(integral, -1000.0f, 1000.0f);

   float difference = error - lastError;
   float derivative;
   if (modulus != 0.0f) {
       if (difference < -modulus) difference += modulus;
       else if (difference > modulus) difference -= modulus;
       derivative = difference / elapsedTime;
   } else {
       derivative = -difference / elapsedTime;
   }
   if (!isfinite(derivative)) derivative = 0.0f;

   lastError = error;

   float correction = kp * error;
   if (ki != 0.0f) correction += ki * integral;
   if (kd != 0.0f) correction -= kd * derivative;

   return absMax == 0.0f ? correction : constrain(correction, -absMax, absMax);
}