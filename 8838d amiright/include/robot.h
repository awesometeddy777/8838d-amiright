#ifndef ROBOTH
#define ROBOTH

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "api.h"
// #include "auton.h"
#include "robot.h"

using namespace pros;

extern pros::Controller controller;

// ----- Defines & Variables ----- //
#define LF_PORT 1
#define LM_PORT 2
#define LB_PORT 3
#define RF_PORT 4
#define RM_PORT 5
#define RB_PORT 6

#define INTAKE1 7
#define INTAKE2 8

#define IMU_PORT 12
#define RACE_DETECTOR_PORT 13

#define VERTICAL_ENCODER 20
#define LATERAL_ENCODER 18

extern bool angleChangerToggle;
extern bool blockBlockToggle;
extern bool lickerToggle;
extern bool descoreToggle;
extern bool raceMechToggle;
extern bool tankToggle;

extern double dRight;
extern double dLeft;
extern double rightDriveD;
extern double leftDriveD;

// ----- Motors ----- //
extern Motor left_motor_front;
extern Motor left_motor_middle;
extern Motor left_motor_back;
extern Motor right_motor_front;
extern Motor right_motor_middle;
extern Motor right_motor_back;

extern Motor intake1;
extern Motor intake2;

extern Motor_Group intake;

// ----- Pneumatics ----- //
extern ADIDigitalOut angleChanger;
extern ADIDigitalOut blockBlock;
extern ADIDigitalOut licker;
extern ADIDigitalOut descore;
extern ADIDigitalOut raceMech;

// ----- Sensors ----- //
extern Imu imu;
extern Optical raceDetector;

extern Rotation verticalEncoder;
extern Rotation lateralEncoder;

#endif