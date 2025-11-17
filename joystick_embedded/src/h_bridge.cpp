#include "h_bridge.h"

void pwm2voltage(PwmValue *pwm, HBridge *voltage, RobotInfo *robotInfo) {
    voltage->leftBridge = pwm->leftBridge.dir * pwm->leftBridge.dt * robotInfo->Vin;
    voltage->rightBridge = pwm->rightBridge.dir * pwm->rightBridge.dt * robotInfo->Vin;
}
