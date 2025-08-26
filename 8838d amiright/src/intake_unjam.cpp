#include "intake_unjam.h"

const int INTAKE_SPEED = 127; // forward command speed
const int UNJAM_SPEED = -100; // reverse briefly
const int UNJAM_TIME = 250;   // ms backwards
const int JAM_TIMEOUT = 300;  // ms stuck before unjam

void intakeControlTask(void *param)
{
    int lastPos = (intake1.get_position() + intake2.get_position()) / 2;
    uint32_t lastMoveTime = millis();

    while (true)
    {
        int power = 0;

        // Controller input
        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1))
        {
            power = INTAKE_SPEED;
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2))
        {
            power = -INTAKE_SPEED;
        }

        intake.move(power); // Move both motors

        if (power > 0)
        {
            int pos = (intake1.get_position() + intake2.get_position()) / 2;
            if (abs(pos - lastPos) > 5)
            {
                lastMoveTime = millis();
            }
            lastPos = pos;

            if (millis() - lastMoveTime > JAM_TIMEOUT)
            {
                intake.move(UNJAM_SPEED); // Reverse both motors
                delay(UNJAM_TIME);
                lastMoveTime = millis();
            }
        }
        else
        {
            lastMoveTime = millis();
            lastPos = (intake1.get_position() + intake2.get_position()) / 2;
        }

        delay(20);
    }
}

void startIntakeTask()
{
    Task intakeTask(intakeControlTask, (void *)"INTAKE",
                    TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                    "Intake Control");
}
