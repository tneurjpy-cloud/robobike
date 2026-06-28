/*//////////////////////////////////////////////////////////////////////////
    control.c
/*//////////////////////////////////////////////////////////////////////////
#include "userdefine.h"

extern TRunState runState;
extern bool sweeping;
static uint32_t chk_start;
int chgCount = 0;
bool autoPilot = false;

// servo control task
// chk_start : start time of the state now
void do_str_cmd_calc()
{
    uint32_t now = millis();

    if (sweeping)
    {

    }
    else if (autoCircling)
    {
        switch (runState)
        {
        case rsOuter: // 円周外 通常走行
            if (autoPilot && (gpio_get_level(IO_2) == 0) && (now - chk_start > 1000))
            {
                if (chgCount < 3)
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

        case rsInner_Correct: // 円周内 修正動作実行
            if (gpio_get_level(IO_2) == 1)
            {
                if (chgCount < 3)
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

        case rsInner_Stable: // 円周内 修正動作終了
            if (gpio_get_level(IO_2) == 1 && (now - chk_start > 1000))
            {
                if (chgCount < 3)
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
    else
    {
        runState = rsOuter;
    }
}

void control_init()
{
    // Check IR sensor install, avoid mirror
    gpio_set_level(IO_1, 1); // IR LED ON
    waitms(1);
    if (gpio_get_level(IO_2) == 1) // check pullup voltage
    {
        gpio_set_level(IO_1, 0); // IR LED OFF
        waitms(1);
        if (gpio_get_level(IO_2) == 0) // check pullup voltage
        {
            autoPilot = true;
        }
    }
    gpio_set_level(IO_1, 0); // IR LED OFF
}
