#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "controller.h"

typedef struct MotorSpeed {
    float leftMotor;
    float rightMotor;
} MotorSpeed;

typedef struct RobotInfo {
    float R;
    float L;
    float Vin;
} RobotInfo;


void robotSpeed2motorSpeed(RobotSpeed *robotSpeed, MotorSpeed *MotorSpeed, RobotInfo *robotInfo);

#endif // _KINEMATICS_H_