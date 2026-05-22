#include "userdefine.h"

extern TRunState runState;
static uint32_t chk_start;
int chgCount = 0;
bool autoPilot = false;

///////////////////////////////////////////////////////////////////
// ABS limitation
void chklimit(float *x, float max)
{
    if (*x < -max)
    {
        *x = -max;
    }
    else if (*x > max)
    {
        *x = max;
    }
}

// servo control task
// chk_start : start time of the state now
void do_str_cmd_calc()
{
    uint32_t now = millis();
    if (autoCircling)
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

//////////////////////////////////////////////////////////////////////////////
// servo control task
void gyroServiceLoop()
{
    void str_pwm_out(float);

    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff; // 偏差
    float w_roll_cmd;

    if (auto_en)                                              // Auto steer enabled
    {                                                         //
        str_dev = str_target - str_out;                       // Steering deviation
        str_dev_diff = (str_dev - last_str_dev) * SV_FRQ;     // Rate of change in deviation
        str_diff_lps =                                        // Low-pass filter for derivative
            saved.str_diff_alph * str_diff_lps                //
            + (1.0f - saved.str_diff_alph) * str_dev_diff;    //
        w_roll_cmd =                                          // Target roll velocity =
            str_dev * saved.gain_str                          //
            + str_diff_lps * saved.gain_str_diff;             // PD control: P*St + D*dSt
    gpio_set_level(IO_1, 0);
            w_roll_dev = w_roll_cmd - IMU_roll();                 // Read IMU and calc roll rate deviation
    gpio_set_level(IO_1, 1);
            str_out -=                                            // Steering servo increment
            w_roll_dev * saved.gain_w_roll * (1.0f / SV_FRQ); // Counter-steering: right steer induces left lean
        last_str_dev = str_dev;                               // Store last deviation
        chklimit(&str_out, STRMAX);                           //
        str_pwm_out(str_out);                                 // +: left steer
    }
    else
    {
        last_str_dev = 0.0f; // reset old values
        str_diff_lps = 0.0f;
        str_out = 0.0f;
        str_pwm_out(str_target); // +: left steer
    gpio_set_level(IO_1, 0);
        if (IMU_getZero())       // auto calibration n sec average
            saved.acc_offset = acc_offset;
    gpio_set_level(IO_1, 1);
    }
}

void control_init()
{
    void waitms(uint32_t);

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
