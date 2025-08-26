#ifndef DRIVEH
#define DRIVEH

#include "main.h"
#include "robot.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

// ----- Functions ----- //
extern void driveVolts(int lspeed, int rspeed, int wt);
extern void driveBrake();
extern void setDriveBrake(motor_brake_mode_e_t mode);
extern void setDriveVelocity(double pct);
extern void resetEncoders();

#endif