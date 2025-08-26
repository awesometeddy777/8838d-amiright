#ifndef PIDH
#define PIDH

#include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "robot.h"
#include "drive.h"

// ----- Defines & Variables ----- //
#define HEADING_KP 8    // 5.25//8.75 //15/////////15
#define HEADING_KI 0    // 0.125//0.115
#define HEADING_KD 9000 // 38 //105 //70 //100 //180///////////////400

#define HEADING_INTEGRAL_KI 0
#define HEADING_MAX_INTEGRAL 0

#define STRAIGHT_KP 8 // 1 //0.7
#define STRAIGHT_KI 0 // 0.001 //0.1
#define STRAIGHT_KD 1 // 1 //8 //change

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

#define TURN_KP 5  // 5.25//8.75
#define TURN_KI 0  // 0.125//0.115
#define TURN_KD 58 // 38 //105 //70

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

extern double error;

// ----- Functions ----- //
extern void setConstants(double kp, double ki, double kd);
extern void setConstants2(double kp, double ki, double kd);
extern double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn);
extern double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn);
extern void driveStraight(int target);
extern void driveStraight2(int target, int speed);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveAngleOld(float targetDistanceInches, float angle, int timeout, int wait_ms = 10,
                          float kp = 6.0, float slew = -1, double maxVel = -1);

#endif