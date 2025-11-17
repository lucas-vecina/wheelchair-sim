#include <math.h>
#include "pwm.h"

void motorSpeed2pwm(MotorSpeed *motorSpeed, PwmValue *pwm) {
    if (motorSpeed->leftMotor > 0) {
        pwm->leftBridge.dir = 1;
    } else {
        pwm->leftBridge.dir = -1;
    }

    if (motorSpeed->rightMotor > 0) {
        pwm->rightBridge.dir = 1;
    } else {
        pwm->rightBridge.dir = -1;
    }

    pwm->leftBridge.dt = 2.0695 * fabs(motorSpeed->leftMotor);
    pwm->rightBridge.dt = 2.0695 * fabs(motorSpeed->rightMotor);

    if (pwm->leftBridge.dt > 1) {
        pwm->leftBridge.dt = 1;
    } else if (pwm->leftBridge.dt < 0) {
        pwm->leftBridge.dt = 0;
    }

    if (pwm->rightBridge.dt > 1) {
        pwm->rightBridge.dt = 1;
    } else if (pwm->rightBridge.dt < 0) {
        pwm->rightBridge.dt = 0;
    }
}
