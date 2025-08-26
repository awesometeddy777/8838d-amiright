#include "drive.h"

// NOTE: This function expects lspeed/rspeed in percent scale [-127..127].
// Internally we convert percent -> motor mV to be consistent with pros Motor API.
void driveVolts(int lspeed, int rspeed, int wt)
{
    // convert percent to mV (approx)
    int l_mV = int((lspeed / 127.0) * 12000.0);
    int r_mV = int((rspeed / 127.0) * 12000.0);

    left_motor_front.move_voltage(l_mV);
    left_motor_middle.move_voltage(l_mV);
    left_motor_back.move_voltage(l_mV);
    right_motor_front.move_voltage(r_mV);
    right_motor_middle.move_voltage(r_mV);
    right_motor_back.move_voltage(r_mV);

    pros::delay(wt);
}

void driveBrake()
{
    left_motor_front.brake();
    left_motor_middle.brake();
    left_motor_back.brake();
    right_motor_front.brake();
    right_motor_middle.brake();
    right_motor_back.brake();
}

void setDriveBrake(motor_brake_mode_e_t mode)
{
    left_motor_front.set_brake_mode(mode);
    left_motor_middle.set_brake_mode(mode);
    left_motor_back.set_brake_mode(mode);
    right_motor_front.set_brake_mode(mode);
    right_motor_middle.set_brake_mode(mode);
    right_motor_back.set_brake_mode(mode);
}

void setDriveVelocity(double pct)
{
    // quick helper: set velocity (percent) for all drive motors
    // Since PROS velocity APIs vary, use move_velocity with the pct as target velocity (user adjustable).
    // If you prefer a different mapping, change this function.
    int v = int(pct);
    left_motor_front.move_velocity(v);
    left_motor_middle.move_velocity(v);
    left_motor_back.move_velocity(v);
    right_motor_front.move_velocity(v);
    right_motor_middle.move_velocity(v);
    right_motor_back.move_velocity(v);
}

void resetEncoders()
{
    left_motor_front.tare_position();
    left_motor_middle.tare_position();
    left_motor_back.tare_position();
    right_motor_front.tare_position();
    right_motor_middle.tare_position();
    right_motor_back.tare_position();
}
