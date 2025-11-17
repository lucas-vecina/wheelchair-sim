#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

typedef struct JoystickOutput {
    unsigned int x;
    unsigned int y;
} JoystickOutput;

typedef struct RobotSpeed {
    float v;
    float w;
} RobotSpeed;


void joystick2robotSpeed(JoystickOutput *joystick, RobotSpeed *robotSpeed);

#endif // _CONTROLLER_H_
