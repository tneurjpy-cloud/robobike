/*//////////////////////////////////////////////////////////////////////////
    control.c
//////////////////////////////////////////////////////////////////////////*/
#include "userdefine.h"

static uint32_t chk_start;
int chgCount = 0;
volatile TRunState runState = rsOuter;
bool IO2;

// servo control task
// chk_start : start time of the state now
void do_str_cmd_calc()
{
    uint32_t now = millis();

    if (!saved.autoCircling)
    {
        runState = rsOuter;
        return;
    }

    IO2 = gpio_get_level(IO_2);
    switch (runState)
    {
    case rsOuter: // 円周外 通常走行
        if ((IO2 == 0) && (now - chk_start > 1000))
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
        if (IO2 == 1)
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
        if (IO2 == 1 && (now - chk_start > 1000))
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
