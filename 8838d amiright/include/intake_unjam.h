#ifndef INTAKEUNJAMH
#define INTAKEUNJAMH

#include "main.h"
#include "robot.h"
#include "drive.h"

extern const int INTAKE_SPEED;
extern const int UNJAM_SPEED;
extern const int UNJAM_TIME;
extern const int JAM_TIMEOUT;

extern void startIntakeTask();
extern void intakeControlTask(void *param);

#endif