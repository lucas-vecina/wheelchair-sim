#ifndef _H_BRIDGE_H_
#define _H_BRIDGE_H_

#include "pwm.h"
#include "kinematics.h"

typedef struct HBridge
{
    float leftBridge;
    float rightBridge;
} HBridge;
 
void pwm2voltage(PwmValue *pwm, HBridge *voltage, RobotInfo *robotInfo);

#endif // _H_BRIDGE_H_