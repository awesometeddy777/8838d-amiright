#include "robot.h"

using namespace pros;

bool angleChangerToggle = false;
bool blockBlockToggle = false;
bool lickerToggle = false;
bool descoreToggle = false;
bool raceMechToggle = false;
bool tankToggle = false;

double dRight;
double dLeft;
double rightDriveD;
double leftDriveD;

pros::Controller controller(E_CONTROLLER_MASTER);

Motor left_motor_front(LF_PORT, E_MOTOR_GEARSET_06, true);
Motor left_motor_middle(LM_PORT, E_MOTOR_GEARSET_06, true);
Motor left_motor_back(LB_PORT, E_MOTOR_GEARSET_06, true);
Motor right_motor_front(RF_PORT, E_MOTOR_GEARSET_06, false);
Motor right_motor_middle(RM_PORT, E_MOTOR_GEARSET_06, false);
Motor right_motor_back(RB_PORT, E_MOTOR_GEARSET_06, false);

Motor intake1(INTAKE1, E_MOTOR_GEARSET_06, false);
Motor intake2(INTAKE2, E_MOTOR_GEARSET_06, false);

Motor_Group intake({intake1, intake2});

ADIDigitalOut angleChanger('A', false);
ADIDigitalOut blockBlock('B', false);
ADIDigitalOut raceMech('C', false); // Rest Assured Color Ejected
ADIDigitalOut licker('D', false);
ADIDigitalOut descore('E', false);

Imu imu(IMU_PORT);
Optical raceDetector(RACE_DETECTOR_PORT);

Rotation verticalEncoder(VERTICAL_ENCODER);
Rotation lateralEncoder(LATERAL_ENCODER);