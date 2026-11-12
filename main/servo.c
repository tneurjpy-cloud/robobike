/*//////////////////////////////////////////////////////////////////////////
    servo.c

Servo control timer chart (4 stages, 4000-us frame, shared by all servos)
2026.11.04 chg. for PCB v6

    == : H level
    __ : L level or stop
    -- : count up running, pause
    +  : start
    |  : fire interrupt

LEDC_CHANNEL_1, 0, 2 --> GPIO_STR, DRV, EX1 (variable duty from 0% to 100%)
ex1sv      s0          s1            s2        s3  pause
____+____________|=============+======______+_________---__+=======
STRsv(FSYNC) s0        s1            s2        s3  pause
____|============+======_______+____________+_________---__+=======
        1000us    500us  500us     1000us       1000+375us
GPIO_STR-->FSYNC external edge input to IMU

sync_timer(gptimer)
CB         cb0           cb1           cb2           cb3
____|-------|-------------|-------------|-------------|_________|-
    0      700us        1700us        2700us        3700us
      FIRSTWT   STGINTRVL     STGINTRVL    STGINTRVL+

CallBackFunctions
cbStrIn: (GPIO external interrupt)
  Start sync_timer to trigger cb0 after FIRSTWT us.

cb0: (gptimer countup interrupt)
  Wake ControlTask.
  Read IMU data and calculate outputs for all servos.
  Write str and mot s1 duty (0-100%).
  Write std servo s0 duty (100%).
  Start the timer interrupt after 1000 us.

cb1: (gptimer countup interrupt)
  Write std s1 duty (0-100%).
  Write str and mot servo s2 duty (0%).
  Start the timer interrupt after 1000 us.

cb2: (gptimer countup interrupt)
  Write std servo s2 duty (0%).
  Start the timer interrupt after 1000 us.

cb3: (gptimer countup interrupt)
  Write str and mot servo s0 duty (100%).
  Stop sync_timer.

//////////////////////////////////////////////////////////////////////////*/

#include "project.h"

static const char TAG[] = "servo";

float str_cmd0;   // deg +-90.0f  +: Left turn
float str_cmd1;   // deg
float str_target; // deg
float str_step;   // deg/cycle
float str_out;    // deg

float mot_cmd;  // % +-100.0f
float mot_step; // %/cycle
float mot_out;  // %

float ex1_cmd;  // deg +-90.0f
float ex1_out;  // deg
float ex1_step; // deg/cycle

TSave saved;
float gain_roll_using;
float *pyaw_coeff;
volatile bool stopServo = false;

const TSave savedefault = {
    .ver = DATAVER,                                        // (int) data format version
    .op_time_s = 0,                                        // (uint32_t) operation time in sec
    .isChecked = false,                                    // isChecked
    .gain_str = 0.030f,                                    // gain_str;
    .gain_str_diff = 0.030f,                               // gain_str_diff
    .gain_w_roll = 14.0f,                                  // gain_w_roll;
    .acc_offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},    // acc_offset
    .acc_dir = {0.0f, 0.0f, 0.0f, 0.0175f, 0.9996f, 0.0f}, // acc_dir
    .str_diff_alph = 0.040f,                               // str_diff_alph
    .str0 = 0,                                             // (int) steering angle neutral R= +deg
    .mot_spd = 50,                                         // (int) motor speed 0-99 (88.5RPM/4.0V, 57.9RPM/SPD=10)
    .ang_std_nut = 20,                                     // (int) stand for start
    .run_coeff = 0.0f,                                     // run-speed feedback coefficient
    .yaw_coeff = 0.010f,                                   // yaw‑rate feedback coefficient
    .str_turn = 40,                                        // (int) str_turn deg
    .str_cmd_speed = 30,                                   // (int) str_cmd_speed deg/sec
    .autoCircling = false,                                 // (bool) autoCircling
    .auto_circ_gain = 0.75f                                // (float) autoCirclingGain
};

static uint32_t firstDelay_us; // us start delay for sync_timer
uint32_t PWM_pause_us = 1;     // us servo_PWM_timer into PLL slave of IMU
uint32_t sv_freq;              // [Hz] Servo control calculation period

float sv_term;
#define STG_INTRVL 1000                                // 1ms / 1MHz
#define PWM_pause_us_NOMINAL (IMU_ODR_TERM * 7 - 4000) // 375us this is only initial value
#define FSYNC_TIME_D_NOMINAL 600

// gpTimer for synchronization
typedef enum // servo sync timer callback step
{
    cb0 = 0,
    cb1,
    cb2,
    cb3,
} TSyncCBStep;

static volatile TSyncCBStep sync_step = cb0;
static gptimer_handle_t sync_timer;
static TaskHandle_t xControlTaskHandle;

// Buffer the variable s2 duty for std (ex1), calculated by the task.
static volatile uint32_t duty_ex1_s2 = 0;

// Preserve the previous values to prevent a 2000-us pulse if the task is late.
uint32_t duty_str_prev = 2048;        // Initial neutral value (equivalent to 1500 us).
static uint32_t duty_mot_prev = 2048; // Initial neutral value (equivalent to 1500 us).
extern void icm426xx_discard_data_SPI();

#include <math.h>

// for sin wave sweep ////////////////////////////////////////////////////
#define DT() (float)(1.0f / sv_freq) // Sampling period (ms)
#define START_FREQ 0.3f              // Start frequency (Hz)
#define END_FREQ 40.0f               // End frequency (Hz)
#define SWEEP_TIME 30.0f             // Sweep duration
#define TWO_PI 6.2831853f            // 2pai rad
#define SWEEP_W 5.0f                 // sweep width(deg)

bool doSweep = false;
float current_time = 0.0f; // sec
float sweep_phase = 0.0f;
float current_freq = START_FREQ;
float freq_multiplier = 1.0f;
float swp_sg = 0.0f;
float swp_ampletude;

///////////////////////////////////////////////////////////////////////////////////
// Float interpolation
float finterp(const float x, const float xt[], const float yt[], const int n)
{
    int i;
    float d0;

    if (x <= xt[0])
    {
        return yt[0];
    }
    else
    {
        i = 1;
        while (x > xt[i] && i < (n - 1))
            i++;
        d0 = xt[i] - xt[i - 1];
        if (d0 <= 0)
        {
            return yt[i - 1];
        }
        else
        {
            return ((yt[i] - yt[i - 1]) * (x - xt[i - 1])) / d0 + yt[i - 1];
        }
    }
}

//////////////////////////////////////////////////////////////////////////
void init_sweep()
{
    sweep_phase = 0.0f;
    current_time = 0.0f;
    doSweep = true;
    current_freq = START_FREQ;
    float total_steps = SWEEP_TIME * sv_freq;
    // K = (END_FREQ / START_FREQ) ^ (1 / total_steps)
    freq_multiplier = powf(END_FREQ / START_FREQ, 1.0f / total_steps);
    swp_ampletude = SWEEP_W;
    if (mot_out != 0.0f)
        swp_ampletude *= 0.01f;
}

/**
 * @brief Called once per servo control period to advance the sweep sine wave.
 * @return Sine-wave value scaled by the configured sweep amplitude.
 * @note The 4-ms (DT) update uses only one multiplication for geometric frequency progression.
 */
static float get_sweep_sine_value(void)
{
    if (!doSweep)
    {
        return 0.0f;
    }

    float delta_phase = TWO_PI * current_freq * sv_term;
    sweep_phase += delta_phase;

    if (sweep_phase >= TWO_PI)
    {
        sweep_phase -= TWO_PI;
    }

    float sine_value = sinf(sweep_phase);

    if (current_freq < END_FREQ)
    {
        current_freq *= freq_multiplier;
    }
    else
    {
        current_freq = END_FREQ; // Hold at the upper limit.
    }

    current_time += sv_term;
    if (current_time >= SWEEP_TIME)
    {
        current_time = 0.0f;
        sweep_phase = 0.0f;
        current_freq = START_FREQ;
        doSweep = false;
        sine_value = 0.0f;
    }

    return sine_value * swp_ampletude;
}

///////////////////////////////////////////////////////////////////
bool auto_en = false;
void auto_enable()
{
    auto_en = true;
}

void auto_disable()
{
    auto_en = false;
}

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

///////////////////////////////////////////////////////////////////
void set_ex1_angle(float angle, float step)
{
    ex1_step = step;
    if (PCBver >= 6)
    {
        angle -= 20.0f; // side stand
    }
    ex1_cmd = angle;
}

void wait_ex1_angle()
{
    ESP_LOGI(TAG, "wait ex1");
    while (ex1_cmd != ex1_out)
    {
        waitTaskms(10);
    }
    ESP_LOGI(TAG, "wait end");
}

/// in ControlTask ///
// Expanded resolution/steps from 1000 to 4000.
static void do_ex1_out()
{
    if (ex1_step == 0.f)
    {
        ex1_out = ex1_cmd;
    }
    else if (ex1_cmd - ex1_out > ex1_step)
    {
        ex1_out += ex1_step;
    }
    else if (ex1_cmd - ex1_out < -ex1_step)
    {
        ex1_out -= ex1_step;
    }
    else
    {
        ex1_out = ex1_cmd;
    }

    float pulsew = (float)((ex1_out * ANG2PULSE) + 1500.0f);

    if (pulsew < (float)PWM_MINLEN)
        pulsew = (float)PWM_MINLEN;
    if (pulsew > (float)PWM_MAXLEN)
        pulsew = (float)PWM_MAXLEN;

    float pulse_s1 = pulsew - (float)PWM_STAGE_LEN;
    duty_ex1_s2 = (int32_t)(pulse_s1 * ((float)PWM_DUTY_100 / (float)PWM_STAGE_LEN));
}

///////////////////////////////////////////////////////////////////
void wait_mot_duty()
{
    while (mot_cmd != mot_out)
    {
        waitTaskms(10);
    }
}

void set_mot_duty(float duty, float step)
{
    mot_cmd = duty;
    mot_step = step;
}

/// in ControlTask ///
static void do_mot_out()
{
    if (mot_step == 0.f)
    {
        mot_out = mot_cmd;
    }
    else if (mot_cmd - mot_out > mot_step)
    {
        mot_out += mot_step;
    }
    else if (mot_cmd - mot_out < -mot_step)
    {
        mot_out -= mot_step;
    }
    else
    {
        mot_out = mot_cmd;
    }

    int32_t mot_pw = (int)(mot_out * (1000.0f / 100.0f)) + 1500;

    if (mot_pw < PWM_MINLEN)
        mot_pw = PWM_MINLEN;
    if (mot_pw > PWM_MAXLEN)
        mot_pw = PWM_MAXLEN;

    uint32_t pulse_s1 = mot_pw - PWM_STAGE_LEN;
    uint32_t duty_s1 = (pulse_s1 * PWM_DUTY_100) / PWM_STAGE_LEN;

    ledc_set_duty(LED_MODE, svch_mot.channel, duty_s1);
    ledc_update_duty(LED_MODE, svch_mot.channel);
    duty_mot_prev = duty_s1;
}

///////////////////////////////////////////////////////////////////
void set_str_cmd(float angle, float step)
{
    str_step = step;
    str_cmd0 = angle;
}

static void str_easing()
{
    extern volatile TRunState runState;
    switch (runState)
    {
    case rsInner_Correct:
        str_cmd1 = str_cmd0 * saved.auto_circ_gain;
        str_target = str_cmd1;
        break;
    default:
        str_cmd1 = str_cmd0;
        if (str_step == 0.f)
        {
            str_target = str_cmd1;
        }
        else if (str_cmd1 - str_target > str_step)
        {
            str_target += str_step;
        }
        else if (str_cmd1 - str_target < -str_step)
        {
            str_target -= str_step;
        }
        else
        {
            str_target = str_cmd1;
        }
    }
}

// servo control task
void str_pwm_out(float angle)
{
    str_out = angle;
    // calculate in float
    float pulsew = (float)((angle + (float)saved.str0) * ANG2PULSE) + 1500.0f;

    if (pulsew < (float)PWM_MINLEN)
        pulsew = (float)PWM_MINLEN;
    if (pulsew > (float)PWM_MAXLEN)
        pulsew = (float)PWM_MAXLEN;

    float pulse_s1 = pulsew - (float)PWM_STAGE_LEN; // usec
    uint32_t duty_s1 = (uint32_t)(pulse_s1 * ((float)PWM_DUTY_100 / (float)PWM_STAGE_LEN));
    ledc_set_duty(LED_MODE, svch_str.channel, duty_s1);
    ledc_update_duty(LED_MODE, svch_str.channel);
    duty_str_prev = duty_s1;
}

void wait_str_angle()
{
    ESP_LOGI(TAG, "wait str");
    while (str_target != str_cmd1)
    {
        waitTaskms(10);
    }
    ESP_LOGI(TAG, "wait str end");
}

////////////////////////////////////////////////////////////////////////////////////
/*
  multiple for gain_roll_using
    | *      *          1.75
    |               *   1.0
    |
    0-------20------50--speed
*/
static const float spd_g_tbl_x[] =
    {20.0f, 50.0f};
static const float spd_g_tbl_y[] =
    {1.75f, 1.0f};

void rollgain_setup()
{
    gain_roll_using = saved.gain_w_roll *
                      finterp(saved.mot_spd, spd_g_tbl_x, spd_g_tbl_y,
                              sizeof(spd_g_tbl_y) / sizeof(spd_g_tbl_y[0]));
}

static void gyroServiceLoop()
{
    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff;
    float w_roll_cmd;

    gpio_set_level(IO_0, 1); // 20~60us to do
    IMU_startRead();
    IMU_getData();
    if (auto_en)
    { // self-balancing control calculation
        str_dev = str_target - str_out;
        str_dev_diff = (str_dev - last_str_dev) * sv_freq;
        str_diff_lps = (1.0f - saved.str_diff_alph) * str_diff_lps + saved.str_diff_alph * str_dev_diff;
        w_roll_cmd = str_dev * saved.gain_str + str_diff_lps * saved.gain_str_diff;
        w_roll_dev = w_roll_cmd - IMU_roll();
        str_out -= w_roll_dev * gain_roll_using * sv_term; // Speed-adaptive gain
        chklimit(&str_out, STRMAX);
        str_pwm_out(str_out);
        last_str_dev = str_dev;
    }
    else
    { // servo responce sweep test
        swp_sg = get_sweep_sine_value();
        last_str_dev = 0.0f;
        str_diff_lps = 0.0f;
        str_out = 0.0f;
        str_pwm_out(str_target + swp_sg);
        if (IMU_getZero())
            saved.acc_offset = acc_offset;
    }

    gpio_set_level(IO_0, 0);
}

// Wakeup on sync_timer_isr/cb0, do str. mot. servo output
static void ControlTask(void *pvParameters)
{
    for (;;)
    {                                            // 90-250us to do
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for servo pulse timing

        gpio_set_level(GPIO_IR_LED, 1); // IR LED ON
        // Preload the previous values in case communication delays the update.
        ledc_set_duty(LED_MODE, svch_str.channel, duty_str_prev);
        ledc_update_duty(LED_MODE, svch_str.channel);
        ledc_set_duty(LED_MODE, svch_mot.channel, duty_mot_prev);
        ledc_update_duty(LED_MODE, svch_mot.channel);

        gyroServiceLoop(); // Read the IMU as early as possible; latch newer data if available in time.

        do_str_cmd_calc();              // auto circling calc
        gpio_set_level(GPIO_IR_LED, 0); // IR LED OFF

        do_mot_out();       // Rear wheel drive
        do_ex1_out();       // Side stand
        str_easing();       // Gradually change the steering target.
        put_control_data(); // Data logging
    }
}

/////////////////////////////////////////////////////////////////////////////
// Only for PCB <= v05
/////////////////////////////////////////////////////////////////////////////
// gptimer interrupt handler 1000us interval state machine.
// If stopServo is true, the servo output will be lowered and the timer will continue to run.
static bool IRAM_ATTR sync_timer_isr_cb(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gptimer_alarm_config_t next_alarm = {0};
    next_alarm.flags.auto_reload_on_alarm = false;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool res = false;

    switch (sync_step)
    {
    case cb0:
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
        if (!stopServo)
        {
            ledc_set_duty(LED_MODE, svch_ex1.channel, PWM_DUTY_100);
            ledc_update_duty(LED_MODE, svch_ex1.channel);
        }
        next_alarm.alarm_count = firstDelay_us + STG_INTRVL;
        gptimer_set_alarm_action(timer, &next_alarm);
        res = true;
        sync_step++;
        break;

    case cb1:
        ledc_set_duty(LED_MODE, svch_str.channel, PWM_DUTY_0);
        ledc_update_duty(LED_MODE, svch_str.channel);
        ledc_set_duty(LED_MODE, svch_mot.channel, PWM_DUTY_0);
        ledc_update_duty(LED_MODE, svch_mot.channel);
        if (!stopServo)
        {
            ledc_set_duty(LED_MODE, svch_ex1.channel, duty_ex1_s2);
            ledc_update_duty(LED_MODE, svch_ex1.channel);
        }
        next_alarm.alarm_count = firstDelay_us + STG_INTRVL * 2;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb2:
        ledc_set_duty(LED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LED_MODE, svch_ex1.channel);

        next_alarm.alarm_count = firstDelay_us + STG_INTRVL * 3;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb3: //Outputs are all L in this stage 
        if (!stopServo)
        {
            ledc_set_duty(LED_MODE, svch_str.channel, PWM_DUTY_100);
            ledc_update_duty(LED_MODE, svch_str.channel);
            ledc_set_duty(LED_MODE, svch_mot.channel, PWM_DUTY_100);
            ledc_update_duty(LED_MODE, svch_mot.channel);
        }
        ledc_set_duty(LED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LED_MODE, svch_ex1.channel);
        sync_step = cb0;
        gptimer_stop(timer); // Stop the timer and wait for an external interrupt.

        //// PLL synchronise to IMU ODR
        extern volatile uint16_t fsync_time_delta;
        if (PCBver <= 5)
        {
            PWM_pause_us = 0;
        }
        else if (fsync_time_delta < FSYNC_TIME_D_NOMINAL - 5)
        {
            PWM_pause_us = PWM_pause_us_NOMINAL - 2; // 375us - 2
        }
        else if (fsync_time_delta < FSYNC_TIME_D_NOMINAL)
        {
            PWM_pause_us = PWM_pause_us_NOMINAL - 1; // 375us - 1
        }
        else if (fsync_time_delta < FSYNC_TIME_D_NOMINAL + 1)
        {
            PWM_pause_us = PWM_pause_us_NOMINAL; // 375us
        }
        else if (fsync_time_delta < FSYNC_TIME_D_NOMINAL + 6)
        {
            PWM_pause_us = PWM_pause_us_NOMINAL + 1; // 375us + 1
        }
        else
        {
            PWM_pause_us = PWM_pause_us_NOMINAL + 2; // 375us + 2
        }
        // LEDC pause time
        ledc_timer_pause(LED_MODE, servo_PWM_timer.timer_num);
        esp_rom_delay_us(PWM_pause_us);
        ledc_timer_resume(LED_MODE, servo_PWM_timer.timer_num);
        break;
    }

    return res;
}

// PulseIn: Servo signal detection ISR; detect the GPIO_STR rising edge at the start of s0.
static gptimer_alarm_config_t first_alarm;
static void IRAM_ATTR gpio_PulseIn_isr_handler(void *arg)
{
    // Respond only while waiting for an external interrupt.
    if (sync_step != cb0)
    {
        return;
    }
    gptimer_set_raw_count(sync_timer, 0);
    gptimer_set_alarm_action(sync_timer, &first_alarm);
    gptimer_start(sync_timer);
}

////////////////////////////////////////////////////////////////////////////
void servo_init()
{
    pyaw_coeff = &saved.yaw_coeff;

    // Create control calculation task
    xTaskCreate(ControlTask, "ControlTask", 2048, NULL, configMAX_PRIORITIES - 1, &xControlTaskHandle);

    // PCB version check
    if (PCBver <= 5)
    { // IMU = I2C
        firstDelay_us = 400;
        sv_freq = 250;
        svch_ex1.gpio_num = GPIO_NUM_9; // PCB v05 for stand servo;
    }
    else
    {                                   // PCB ver. >= 6 IMU = SPI, GPIO_FSYNC
        firstDelay_us = 700;            //
        sv_freq = 228;                  // Synchronize the 4-ms cycle with the IMU ODR of 228 Hz.
        svch_ex1.gpio_num = GPIO_NUM_6; // PCB v06 or later;
    }
    sv_term = 1.0f / (float)sv_freq;
    first_alarm.alarm_count = firstDelay_us;
    first_alarm.flags.auto_reload_on_alarm = false;

    // gptimer setup
    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ, // 1MHz= 1us resolution
    };
    gptimer_new_timer(&timer_config, &sync_timer);
    const gptimer_event_callbacks_t cbs = {
        .on_alarm = sync_timer_isr_cb,
    };
    gptimer_register_event_callbacks(sync_timer, &cbs, NULL);
    gptimer_enable(sync_timer);

    // Attach a GPIO INPUT interrupt directly to the PWM OUTPUT pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    io_conf.pin_bit_mask = (1ULL << svch_str.gpio_num);

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(svch_str.gpio_num, gpio_PulseIn_isr_handler, NULL);

    // PWM timer setup
    ledc_timer_config(&servo_PWM_timer);
    ledc_channel_config(&svch_mot);
    ledc_channel_config(&svch_str);
    ledc_channel_config(&svch_ex1);

    // servo initial value
    set_mot_duty(0.0f, 0.0f);
    set_str_cmd(0.0f, 0.0f);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.0f);

    auto_disable();
    ESP_LOGI(TAG, "3 servos integrated into INTERLEAVED SYNC mode.");
}
