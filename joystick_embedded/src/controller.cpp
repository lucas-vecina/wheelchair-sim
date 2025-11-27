#include <math.h>
#include "controller.h"

void joystick2robotSpeed(JoystickOutput *joystick, RobotSpeed *robotSpeed) {
    robotSpeed->v = 0.0010 * joystick->y - 0.5000;
    robotSpeed->w = 0.0010 * joystick->x - 0.5236;

    if (fabs(robotSpeed->v) <= 0.01) {
        robotSpeed->v = 0.;
    } 
    if (fabs(robotSpeed->w) <= 0.01) {
        robotSpeed->w = 0.;
    }
}
