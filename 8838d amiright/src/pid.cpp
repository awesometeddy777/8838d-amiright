#include "pid.h"
// #include "autons.h"`
#include <iostream>

using namespace pros;
using namespace c;
using namespace std;

double trueTarget = 0;

double hKp;
double hKi;
double hKd;
double error;
double prevError;
double integral;
double derivative;
int time2;
double power;

double hKp2;
double hKi2;
double hKd2;
double error2;
double prevError2;
double integral2;
double derivative2;
double power2;

void setConstants(double kp, double ki, double kd)
{
    hKp = kp;
    hKi = ki;
    hKd = kd;
}

void setConstants2(double kp, double ki, double kd)
{
    hKp2 = kp;
    hKi2 = ki;
    hKd2 = kd;
}

double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{ // basically tuning i here

    int integral;

    prevError = error;
    error = target - input;

    if (abs(error) < integralKi)
    {
        integral += error;
    }
    else
    {
        integral = 0;
    }

    if (integral >= 0)
    {
        integral = min(integral, maxIntegral); // min means take whichever value is smaller btwn integral and maxI
        // integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    }
    else
    {
        integral = max(integral, -maxIntegral); // same thing but negative max
    }

    derivative = error - prevError;

    power = (hKp * error) + (hKi * integral) + (hKd * derivative);

    return power;
}

double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{ // basically tuning i here
    int integral2;
    prevError2 = error2;
    error2 = target - input;

    if (std::abs(error2) < integralKi)
    {
        integral2 += error2;
    }
    else
    {
        integral2 = 0;
    }

    if (integral2 >= 0)
    {
        integral2 = std::min(integral2, maxIntegral); // min means take whichever value is smaller btwn integral and maxI
        // integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    }
    else
    {
        integral2 = std::max(integral2, -maxIntegral); // same thing but negative max
    }

    derivative2 = error2 - prevError2;

    power2 = (hKp * error2) + (hKi * integral2) + (hKd * derivative2);

    return power2;
}

void driveStraight(int target)
{

    imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // controller Display Cycle
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = 0;
    x = double(abs(target));

    // timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; //Tune with Desmos

    resetEncoders();
    while (true)
    {
        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        // if(longValues){
        //     setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        // } else {
        //     setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        // }
        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }
        // errorp = abs(target - encoderAvg);
        heading_error = 0;
        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
    }
    driveBrake();
}
void driveStraightNew(int target)
{

    imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = 0;
    x = double(abs(target));

    resetEncoders();
    while (true)
    {
        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }
        heading_error = 0;

        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "vkp: %f           ", float(hKp));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
    }
    driveBrake();
}

/*
// void driveStraight2(int target, int speedPct)
// {
//     int timeout = 5000; // ms
//     int count = 0;
//     time2 = 0;

//     double voltage = 0;
//     double encoderAvg = 0;
//     double heading_error = 0;

//     // normalize target heading
//     trueTarget = imu.get_heading();
//     if (trueTarget > 180)
//     {
//         trueTarget -= 360;
//     }

//     setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
//     setConstants2(HEADING_KP, HEADING_KI, HEADING_KD);

//     resetEncoders();
//     encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

//     while (true)
//     {
//         // update encoder position
//         encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

//         // adjust PID constants near the target
//         if (fabs(target - encoderAvg) < 25)
//         {
//             setConstants(2.5, 0, 0); // simple proportional near stop
//         }
//         else
//         {
//             setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
//         }

//         // base forward PID (distance)
//         voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

//         // current IMU heading (convert to ±180)
//         double position = imu.get_heading();
//         if (position > 180)
//             position -= 360;

//         // heading correction PID
//         heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

//         // scale voltage to speed percentage
//         double speedFactor = double(speedPct) / 100.0;
//         if (voltage > 127 * speedFactor)
//             voltage = 127 * speedFactor;
//         if (voltage < -127 * speedFactor)
//             voltage = -127 * speedFactor;

//         // drive with correction
//         driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);

//         // exit conditions
//         if (fabs(target - encoderAvg) <= 4)
//             count++;
//         if (count >= 8 || time2 > timeout)
//             break;

//         // debug to controller
//         if (time2 % 100 == 0)
//         {
//             controller.print(0, 0, "Err: %.1f HeadErr: %.1f", error, heading_error);
//         }

//         pros::delay(10);
//         time2 += 10;
//     }

//     driveBrake();
// }
*/

void driveStraight2(int target, int speed, int timeout = 5000)
{

    bool over = false;
    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if (trueTarget > 180)
    {
        trueTarget = trueTarget - 360;
    }

    resetEncoders();

    while (true)
    {

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((trueTarget < 0) && (position > 0))
        {
            if ((position - trueTarget) >= 180)
            {
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            }
        }
        else if ((trueTarget > 0) && (position < 0))
        {
            if ((trueTarget - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;
        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127 * double(speed) / 100.0)
        {
            voltage = 127 * double(speed) / 100.0;
        }
        else if (voltage < -127 * double(speed) / 100.0)
        {
            voltage = -127 * double(speed) / 100.0;
        }

        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 4)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "true: %f           ", float(trueTarget));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
        // hi
    }
    driveBrake();
}

void driveTurn(int target)
{ // target is inputted in autons
    double voltage;
    double position;
    int count = 0;
    time2 = 0;

    setConstants(TURN_KP, TURN_KI, TURN_KD);

    int timeout = 30000;
    double variKP = 0;
    double variKD = 0;
    double x = 0;

    // if(abs(target>=25)){
    // setConstants(TURN_KP, TURN_KI, variKD);
    // } else if(mogoValues == false) {
    // setConstants(5, TURN_KI, 90);
    // }

    // setConstants(variKP, TURN_KI, variKD);
    setConstants(TURN_KP, TURN_KI, TURN_KD);
    imu.tare_heading();

    while (true)
    {
        position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if (abs(error) <= 2)
        {
            setConstants(11, 0, 0);
        }
        else
        {
            setConstants(TURN_KP, TURN_KI, TURN_KD);
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        driveVolts(voltage, -voltage, 0);
        if (fabs(target - position) <= 0.5)
            count++;
        if (count >= 20 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);
    }
    driveBrake();
}
/*
#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "drive.h"

using namespace pros;
using namespace std;

double trueTarget = 0;

double hKp = 0;
double hKi = 0;
double hKd = 0;
double error = 0;
double prevError = 0;
double derivative = 0;
int time2 = 0;
double power = 0;

double hKp2 = 0;
double hKi2 = 0;
double hKd2 = 0;
double error2 = 0;
double prevError2 = 0;
double derivative2 = 0;
double power2 = 0;

void setConstants(double kp, double ki, double kd)
{
    hKp = kp;
    hKi = ki;
    hKd = kd;
}

void setConstants2(double kp, double ki, double kd)
{
    hKp2 = kp;
    hKi2 = ki;
    hKd2 = kd;
}

// Fixed: use persistent integral accumulators and avoid shadowing/uninitialized variables
double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{
    static double integral_accum = 0.0;

    prevError = error;
    error = target - input;

    if (std::abs(error) < integralKi)
    {
        integral_accum += error;
    }
    else
    {
        integral_accum = 0.0;
    }

    if (integral_accum >= 0.0)
    {
        integral_accum = std::min(integral_accum, double(maxIntegral));
    }
    else
    {
        integral_accum = std::max(integral_accum, -double(maxIntegral));
    }

    derivative = error - prevError;

    power = (hKp * error) + (hKi * integral_accum) + (hKd * derivative);

    return power;
}

double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{
    static double integral_accum2 = 0.0;

    prevError2 = error2;
    error2 = target - input;

    if (std::abs(error2) < integralKi)
    {
        integral_accum2 += error2;
    }
    else
    {
        integral_accum2 = 0.0;
    }

    if (integral_accum2 >= 0.0)
    {
        integral_accum2 = std::min(integral_accum2, double(maxIntegral));
    }
    else
    {
        integral_accum2 = std::max(integral_accum2, -double(maxIntegral));
    }

    derivative2 = error2 - prevError2;

    // Fixed: use second PID constants
    power2 = (hKp2 * error2) + (hKi2 * integral_accum2) + (hKd2 * derivative2);

    return power2;
}

void driveStraight(int target)
{
    imu.tare();
    double voltage = 0;
    double encoderAvg = 0;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = double(abs(target));

    resetEncoders();
    // initialize encoderAvg before use
    encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

    while (true)
    {
        // recompute encoderAvg at start of loop
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, int(STRAIGHT_MAX_INTEGRAL));

        double position = imu.get_heading();
        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }

        // IMPORTANT: do not zero heading_error here — use it for correction
        driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);

        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        pros::delay(10);
        time2 += 10;
    }
    driveBrake();
}

void driveStraight2(int target, int speed)
{
    int timeout = 5000;

    double voltage = 0;
    double encoderAvg = 0;
    int count = 0;
    double heading_error = 0;
    time2 = 0;

    if (trueTarget > 180)
    {
        trueTarget = trueTarget - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    resetEncoders();
    encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

    while (true)
    {
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, int(STRAIGHT_MAX_INTEGRAL));

        double position = imu.get_heading();
        if (position > 180)
        {
            position = position - 360;
        }

        if ((trueTarget < 0) && (position > 0))
        {
            if ((position - trueTarget) >= 180)
            {
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            }
        }
        else if ((trueTarget > 0) && (position < 0))
        {
            if ((trueTarget - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        double speedFactor = double(speed) / 100.0;
        if (voltage > 127 * speedFactor)
        {
            voltage = 127 * speedFactor;
        }
        else if (voltage < -127 * speedFactor)
        {
            voltage = -127 * speedFactor;
        }

        driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);
        if (abs(target - encoderAvg) <= 4)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "true: %f           ", float(trueTarget));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        pros::delay(10);
        time2 += 10;
    }
    driveBrake();
}*/