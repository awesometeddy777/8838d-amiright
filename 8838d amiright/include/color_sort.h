#ifndef COLORSORTH
#define COLORSORTH

#include "api.h"
#include "main.h"
#include "pros/api_legacy.h"
#include "robot.h"
#include "drive.h"

// Hue ranges for Color Sensor
#define RED1_HUE_MIN 0
#define RED1_HUE_MAX 20
#define RED2_HUE_MIN 320
#define RED2_HUE_MAX 360
#define BLUE_HUE_MIN 180
#define BLUE_HUE_MAX 275
#define TRAPDOOR_TIME 350
#define MIN_PROXIMITY 40

extern bool wantRed;
// extern bool blockHandled;
extern int hue;
extern int prox;

void colorSortTask(void *param);
void startColorSortTask();

#endif