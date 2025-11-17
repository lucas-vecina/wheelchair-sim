#include "kinematics.h"

void robotSpeed2motorSpeed(RobotSpeed *robotSpeed, MotorSpeed *motorSpeed, RobotInfo *robotInfo) {
    motorSpeed->leftMotor = (1/robotInfo->R) * robotSpeed->v - (robotInfo->L/(2*robotInfo->R)) * robotSpeed->w;
    motorSpeed->rightMotor = (1/robotInfo->R) * robotSpeed->v + (robotInfo->L/(2*robotInfo->R)) * robotSpeed->w;
}
