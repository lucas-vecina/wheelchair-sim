#include "controller.h"

void joystick2robotSpeed(JoystickOutput *joystick, RobotSpeed *robotSpeed) {
    robotSpeed->v = 0.0003 * joystick->y - 0.145;
    robotSpeed->w = 0.0008 * joystick->x - 0.4142;
}
