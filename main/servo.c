/*//////////////////////////////////////////////////////////////////////////
    servo.c
//////////////////////////////////////////////////////////////////////////*/
#include "userdefine.h"

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

bool autoCircling = true;
TRunState runState = rsOuter;

TSave saved;
float *pyaw_coeff;

const TSave savedefault = {
    DATAVER,                                      // (int) data format version
    0,                                            // (uint32_t) operation time in sec
    false,                                        // isChecked
    0.030f,                                       // gain_str;
    0.030f,                                       // gain_str_diff
    12.0f,                                        // gain_w_roll;
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},         // acc_offset
    {0.0f, 0.0f, 0.0f, 0.0175f, 0.9996f, 0.000f}, // acc_dir
    0.040f,                                       // str_diff_alph
    0,                                            // (int) steering angle neutral R= +deg
    60,                                           // (int) motor speed 0-99 (88.5RPM/4.0V, 57.9RPM/SPD=10)
    20,                                           // (int) stand for start
    0.0f,                                         // run-speed feedback coefficient
    0.011f,                                       // yaw‑rate feedback coefficient
    40,                                           // (int) str_turn deg
    30,                                           // (int) str_cmd_speed deg/sec
    0xFFFFFFFF                                    // (uint32_t) CRC
};

//// R/C servo pulse width making
#define USEC2LEDCDUTY(x) (((x) * 16384) / (1000000 / SV_FRQ)) // LEDC_TIMER_14_BIT 2^14
#define DELAYTIME_RISING 650                                  // us Delay time for rising edge

static const ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT, // 16384 steps for 100% duty
    .timer_num = LEDC_TIMER_0,
    .freq_hz = SV_FRQ,
    .clk_cfg = LEDC_AUTO_CLK};

static ledc_channel_config_t svch_mot = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_DRV,
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = USEC2LEDCDUTY(DELAYTIME_RISING + 500)};

static ledc_channel_config_t svch_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR,
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = USEC2LEDCDUTY(DELAYTIME_RISING)};

static ledc_channel_config_t svch_ex1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_EX1,
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = USEC2LEDCDUTY(DELAYTIME_RISING + 1000)};

///////////////////////////////////////////////////////////////////
/// in task web-server ///
// angle:+-deg  step: deg/cycle
void set_str_cmd(float angle, float step)
{
    str_step = step;
    str_cmd0 = angle;
}

// servo control task ///
// set_str_cmd(angle) -> str_cmd0 -> str_cmd1 -> str_target -> str_cal
static void str_easing()
{
    switch (runState)
    {
    case rsInner_Correct:
        str_cmd1 = str_cmd0 * AUTOCORRECTRATE;
        str_target = str_cmd1; // without easing
        break;

    default:
        str_cmd1 = str_cmd0;
        if (str_step == 0.f)
        {
            str_target = str_cmd1;
        }
        else if (str_cmd1 - str_target > str_step)
        {
            str_target += str_step; // easing
        }
        else if (str_cmd1 - str_target < -str_step)
        {
            str_target -= str_step; // easing
        }
        else
        {
            str_target = str_cmd1;
        }
    }
}

/// servo control task ///
// angle: deg -90/+90 ang + = right turn
void str_pwm_out(float angle)
{
    str_out = angle;
    int32_t pulsew = (int32_t)((angle + (float)saved.str0) * ANG2PULSE) + 1500;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel,
                  USEC2LEDCDUTY(pulsew));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
}

/// in task web-server or any
void wait_str_angle()
{
    ESP_LOGI(TAG, "wait str");
    while (str_target != str_cmd1)
    {
        waitTaskms(10);
    }
    ESP_LOGI(TAG, "wait str end");
}

///////////////////////////////////////////////////////////////////
/// any task ///
// angle +-90deg , step deg/cycle
void set_ex1_angle(float angle, float step)
{
    ex1_step = step;
    ex1_cmd = angle;
}

/// in task web-server ///
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

    int32_t out = (int32_t)(ex1_out * ANG2PULSE) + 1500;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel,
                  USEC2LEDCDUTY(out));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);
}

/// Continuous rotation motor servo using for drive /////////////////////////
/// in task web-server ///
void wait_mot_duty()
{
    while (mot_cmd != mot_out)
    {
        waitTaskms(10);
    }
}

// duty = -100.0(500us) / +100.0(2500us)
// static uint32_t stoptime;
// static bool timer_ON = false;
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

    int32_t mot_pw = (int)(mot_out * (1000.0f / 100.0f)) + 1500; // mot_cmd = +-100
    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel,
                  USEC2LEDCDUTY(mot_pw));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
}

///////////////////////////////////////////////////////////////////////
bool auto_en = false;
void auto_enable()
{
    auto_en = true;
}

void auto_disable()
{
    auto_en = false;
}

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

//////////////////////////////////////////////////////////////////////////////
// servo control task
void gyroServiceLoop()
{
    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff; // 偏差
    float w_roll_cmd;

    IMU_startRead();                                          // Start I2C read of IMU data, will be available in 10-20ms
    if (auto_en)                                              // Auto steer enabled
    {                                                         //
        str_dev = str_target - str_out;                       // Steering deviation
        str_dev_diff = (str_dev - last_str_dev) * SV_FRQ;     // Rate of change in deviation
        str_diff_lps =                                        // Low-pass filter for derivative
            (1.0f - saved.str_diff_alph) * str_diff_lps       //
            + saved.str_diff_alph * str_dev_diff;             //
        w_roll_cmd =                                          // Target roll velocity =
            str_dev * saved.gain_str                          //
            + str_diff_lps * saved.gain_str_diff;             // PD control: P*St + D*dSt
        w_roll_dev = w_roll_cmd - IMU_roll();                 // Read IMU and calc roll rate deviation
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
        if (IMU_getZero())       // auto calibration n sec average
            saved.acc_offset = acc_offset;
    }
}

//// STR servo PWM rising edge trigger ////////////////////////////
static TaskHandle_t xControlTaskHandle;
static gptimer_handle_t sync_timer;
volatile bool PWM_phase_sync = true;

// Task synchronized with the servo PWM signal
static void ControlTask(void *pvParameters)
{
    acc_offset = saved.acc_offset;
    for (;;)
    { // Wait for Notify from sync_timer callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(IO_1, 1);       // IR LED ON
        gyroServiceLoop();             // Read IMU, calc control, update servo outputs
        put_control_data();            // Send control data to web-server task
        do_ex1_out();                  // Side Stand calc.
        do_mot_out();                  // Motor drive calc.
        do_str_cmd_calc();             // Area detection calc.
        str_easing();                  // Easing for steering command
        gpio_set_level(IO_1, 0);       // IR LED OFF
    }
}

////////////////////////////////////////////////////////////////////////////
/// Master Sync Callback ///
static bool IRAM_ATTR sync_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    if (PWM_phase_sync)
    {
        ledc_timer_rst(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
        PWM_phase_sync = false;
    }
    vTaskNotifyGiveFromISR(xControlTaskHandle, NULL);
    return true; // return true to auto-reload the timer
}

////////////////////////////////////////////////////////////////////////////
/// initialization of servo control and Master Sync ///
void servo_init()
{
    pyaw_coeff = &saved.yaw_coeff;

    // LEDC setup for servo control
    ledc_timer_config(&servo_timer);
    ledc_channel_config(&svch_mot);
    ledc_channel_config(&svch_str);
    ledc_channel_config(&svch_ex1);

    control_init();
    xTaskCreate(ControlTask, "ControlTask", 2048, NULL, configMAX_PRIORITIES - 1, &xControlTaskHandle);

    // initialize GPTIMER for Master Sync
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz (1us単位)
    };
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / SV_FRQ,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true, // ESP-IDF v6.x
        },
    };
    gptimer_new_timer(&timer_config, &sync_timer);
    gptimer_set_alarm_action(sync_timer, &alarm_config);
    gptimer_event_callbacks_t cbs = {
        .on_alarm = sync_timer_callback,
    };
    gptimer_register_event_callbacks(sync_timer, &cbs, NULL);
    gptimer_enable(sync_timer);
    gptimer_start(sync_timer);

    // Set initial servo positions and disable outputs
    auto_disable();
    str_pwm_out(0);
    set_mot_duty(0, 0.0f);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.0f);
    ESP_LOGI(TAG, "Servo initialized with INTERNAL MASTER SYNC mode.");
}