#include "main.h"
#include "robot.h"
#include "autons.h"
#include "api.h"
#include "display.h"
#include "drive.h"
#include "pid.h"
#include "intake_unjam.h"
#include "color_sort.h"

using namespace std;
using namespace pros;

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		lcd::set_text(2, "I was pressed!");
	}
	else
	{
		lcd::clear_line(2);
	}
}

void screenTask(void *)
{
	while (true)
	{
		// keep screens running in their own task so competition_initialize can return
		controllerScreen();
		brain_screen();
		pros::delay(50);
	}
}

void initialize()
{
	lcd::initialize();
	lcd::set_text(1, "Hello PROS User!");

	lcd::register_btn1_cb(on_center_button);

	startIntakeTask();
	startColorSortTask();
}

void disabled() {}

void competition_initialize()
{
	drawLogo();
	// spawn the screen updater but don't block returning from competition_initialize
	pros::Task screenUpdater(screenTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "screenUpdater");
	// give the task a moment
	pros::delay(50);
}

void autonomous()
{
	runAuton();
}

void opcontrol()
{
	while (true)
	{
		// Motor Controls
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X))
		{
			tankToggle = !tankToggle;
		}

		if (tankToggle)
		{
			// read axes as doubles in range [-127,127]
			double dLeftRaw = controller.get_analog(ANALOG_LEFT_Y);
			double dRightRaw = controller.get_analog(ANALOG_RIGHT_Y);

			// cubic-ish deadband shaping but preserve sign: ((|x|*x)/127)
			double leftScaled = (fabs(dLeftRaw) * dLeftRaw) / 127.0;
			double rightScaled = (fabs(dRightRaw) * dRightRaw) / 127.0;

			// convert to integer percent in [-127..127]
			int leftPercent = int(round(leftScaled * 127.0));
			int rightPercent = int(round(rightScaled * 127.0));

			driveVolts(leftPercent, rightPercent, 0); // driveVolts now expects percent
		}
		else
		{
			int power = controller.get_analog(ANALOG_LEFT_Y); // [-127..127]
			int RX = controller.get_analog(ANALOG_RIGHT_X);	  // [-127..127]

			// cubic turn shaping (faster than pow)
			int turn = (RX * RX * RX) / (127 * 127);

			int left = power + turn;
			int right = power - turn;

			// clamp to [-127..127]
			if (left > 127)
				left = 127;
			if (left < -127)
				left = -127;
			if (right > 127)
				right = 127;
			if (right < -127)
				right = -127;

			driveVolts(left, right, 0);
		}

		// Pneumatics
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1))
		{
			angleChangerToggle = !angleChangerToggle;
			angleChanger.set_value(angleChangerToggle);
		}

		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2))
		{
			blockBlockToggle = !blockBlockToggle;
			blockBlock.set_value(blockBlockToggle);
		}

		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B))
		{
			lickerToggle = !lickerToggle;
			licker.set_value(lickerToggle);
		}

		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN))
		{
			descoreToggle = !descoreToggle;
			descore.set_value(descoreToggle);
		}

		// Auton Select
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
		{
			lastAuton();
		}
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP))
		{
			nextAuton();
		}
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
		{
			runAuton();
		}

		// Important: yield CPU
		pros::delay(10);
	}
}
