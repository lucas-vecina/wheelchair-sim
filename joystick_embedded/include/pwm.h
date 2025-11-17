#ifndef _PWM_H_
#define _PWM_H_

#include "kinematics.h"

typedef struct PwmValue {
    struct Bridge {
        signed char dir;
        float dt;
    };

    struct Bridge leftBridge;
    struct Bridge rightBridge;
} PwmValue;

void motorSpeed2pwm(MotorSpeed *motorSpeed, PwmValue *pwm);

#endif // _PWM_H_
