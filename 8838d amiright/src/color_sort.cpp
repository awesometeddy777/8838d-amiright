#include "color_sort.h"

bool wantRed = true; // start by sorting red
// bool blockHandled = false;
int hue;
int prox;

void colorSortTask(void *param)
{
    raceDetector.set_led_pwm(100);
    while (true)
    {
        if (controller.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
        {
            wantRed = !wantRed;
            delay(300); // debounce
        }

        int prox = raceDetector.get_proximity();
        if (prox > MIN_PROXIMITY)
        { // only act if a block is detected
            int hue = raceDetector.get_hue();

            if (wantRed && (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX))
            {
                // if (!blockHandled)
                // {
                raceMech.set_value(true);
                delay(TRAPDOOR_TIME);
                raceMech.set_value(false);
                // blockHandled = true;
                // }
            }
            else if (!wantRed && ((hue >= RED1_HUE_MIN && hue <= RED1_HUE_MAX) || (hue >= RED2_HUE_MIN && hue <= RED2_HUE_MAX)))
            {
                // if (!blockHandled)
                // {
                raceMech.set_value(true);
                delay(TRAPDOOR_TIME);
                raceMech.set_value(false);
                // blockHandled = true;
                // }
            }
        }
        else
        {
            // blockHandled = false; // reset when nothing is detected
        }

        delay(1);
    }
}

void startColorSortTask()
{
    Task colorTask(colorSortTask, (void *)"COLOR_SORT",
                   TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                   "Color Sort Control");
}
