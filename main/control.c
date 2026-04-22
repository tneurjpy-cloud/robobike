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
// chk_start : ステートの開始時刻
void do_str_cmd_calc()
{
    if (autoCircling)
    {
        switch (runState)
        {
        case rsOuter: // 円周外 通常走行
            if (autoPilot && (gpio_get_level(IO_2) == 0) && (millis - chk_start > 1000))
            {
                if (chgCount < 3)
                {
                    ++chgCount;
                }
                else
                {
                    runState = rsInner_Correct;
                    chk_start = millis;
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
                    chk_start = millis;
                    chgCount = 0;
                }
            }

            if ((millis - chk_start) >= AUTOCORRECTTIME)
            {
                runState = rsInner_Stable;
                chk_start = millis;
                chgCount = 0;
            }
            break;

        case rsInner_Stable: // 円周内 修正動作終了
            if (gpio_get_level(IO_2) == 1 && (millis - chk_start > 1000))
            {
                if (chgCount < 3)
                {
                    ++chgCount;
                }
                else
                {
                    runState = rsOuter;
                    chk_start = millis;
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
// 307-653us to do gyroServiceLoop()
//   -> pulse ->  USEC2LEDCDUTY(pulse)
void gyroServiceLoop()
{
    void str_pwm_out(float);

    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff; // 偏差
    float w_roll_cmd;

    if (auto_en)
    {
        str_dev = str_cmd2 - str_out;                             // ステア偏差
        str_dev_diff = (str_dev - last_str_dev) * SERVO_FREQ;     // 偏差変化率
        str_diff_lps =                                            // lopass
            saved.str_diff_alph * str_diff_lps                    //
            + (1.0f - saved.str_diff_alph) * str_dev_diff;        //
        w_roll_cmd =                                              // ロール角目標値 =
            -str_dev * saved.gain_str                             //
            - str_diff_lps * saved.gain_str_diff;                 // P*St + D*dSt
        w_roll_dev = w_roll_cmd - IMU_roll();                     // IMU読出し,角速度偏差算出
        str_out +=                                                // ステアサーボ増分
            w_roll_dev * saved.gain_w_roll * (1.0f / SERVO_FREQ); //
        last_str_dev = str_dev;                                   // 偏差記憶
        chklimit(&str_out, STRMAX);
        str_pwm_out(str_out); // + = right turn
    }
    else
    {
        last_str_dev = 0.0f; // reset old values
        str_diff_lps = 0.0f;
        str_out = 0.0f;
        str_pwm_out(str_cmd2); // + = right turn
        IMU_getZero();         // auto calibration n sec average
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
    
