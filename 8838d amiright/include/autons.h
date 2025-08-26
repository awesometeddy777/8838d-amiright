#ifndef AUTONS
#define AUTONS

#include "main.h"
#include "robot.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include <iostream>
#include "map"
#include "api.h"
#include "pid.h"
#include "drive.h"

// ----- Defines & Variables ----- //
enum autonSelect
{
    redLeftAuton,
    redRightAuton,
    blueLeftAuton,
    blueRightAuton
};
extern autonSelect currentAuton;

// ----- Functions ----- //
extern void redLeft();
extern void redRight();
extern void blueLeft();
extern void blueRight();

extern void nextAuton();
extern void lastAuton();
extern void runAuton();
extern const char *getAutonName(autonSelect currentA);

#endif