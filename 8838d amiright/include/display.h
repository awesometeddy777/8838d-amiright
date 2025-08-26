#ifndef DISPLAYH
#define DISPLAYH
#include "api.h"
#include "main.h"
#include "robot.h"
#include "autons.h"
#include <stdlib.h>
#include "pid.h"

// ----- Functions ----- //
extern void displayMotorStatus(Motor &m, const char *name, int line);
extern void brain_screen();
extern void controllerScreen();
extern void drawLogo();

#endif