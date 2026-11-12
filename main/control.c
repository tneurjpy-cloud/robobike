/*//////////////////////////////////////////////////////////////////////////
    control.c
//////////////////////////////////////////////////////////////////////////*/
#include "project.h"

static uint32_t chk_start;
int chgCount = 0;
volatile TRunState runState = rsOuter;

// servo control task
// chk_start : start time of the state now
void do_str_cmd_calc()
{
    uint32_t now = millis();
    bool inCirc = !gpio_get_level(GPIO_IR_PTR);

    if (!saved.autoCircling)
    {
        runState = rsOuter;
        return;
    }

    switch (runState)
    {
    case rsOuter: // Outside the circle: normal riding
        if (inCirc && (now - chk_start > 1000))
        {
            if (chgCount < 4)
            {
                ++chgCount;
            }
            else
            {
                runState = rsInner_Correct;
                chk_start = now;
                chgCount = 0;
            }
        }
        else
        {
            chgCount = 0;
        }
        break;

    case rsInner_Correct: // Inside the circle: apply correction
        if (!inCirc)
        { // Outside the circle
            if (chgCount < 4)
            {
                ++chgCount;
            }
            else
            {
                runState = rsOuter;
                chk_start = now;
                chgCount = 0;
            }
        }

        if ((now - chk_start) >= AUTOCORRECTTIME)
        {
            runState = rsInner_Stable;
            chk_start = now;
            chgCount = 0;
        }
        break;

    case rsInner_Stable: // Inside the circle: correction complete
        if (!inCirc && (now - chk_start > 1000))
        {
            if (chgCount < 4)
            {
                ++chgCount;
            }
            else
            {
                runState = rsOuter;
                chk_start = now;
                chgCount = 0;
            }
        }
        else
        {
            chgCount = 0;
        }
        break;
    }
}
