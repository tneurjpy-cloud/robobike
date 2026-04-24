/*//////////////////////////////////////////////////////////////////////////
    servo.c 転倒防止制御の計算を含むサーボ出力全般を担当

ISRの極小化: 割り込みスタックやリエントラント性の問題を回避。
タスクでの演算: スタック消費の局所化とスケジューリングの安定化。
定数化による最適化: 実行時の除算排除。
200Hz周期: CPU負荷率約10%という高いマージンの確保。

これらにより、ESP32-C3というリソースに制約のあるチップで、
非常に決定論的（Deterministic）で堅牢な制御システムが構築できていると言えます。
//////////////////////////////////////////////////////////////////////////*/
#include "userdefine.h"

static const char TAG[] = "servo";

#define USEC2LEDCDUTY(x) ((x) * 16384 / (1000000 / SERVO_FREQ))
#define DELAY_TIME_USEC 3600

float str_cmd0; // deg +-90.0f
float str_cmd1; // deg
float str_cmd2; // deg
float str_step; // deg/cycle
float str_out;  // deg
TRunState runState = rsOuter;

float mot_cmd;  // % +-100.0f
float mot_step; // %/cycle
float mot_out;  //

float ex1_cmd;  // deg +-90.0f
float ex1_out;  // deg
float ex1_step; // deg/cycle

bool autoCircling = true;

TSave saved;
float *pyaw_coeff;

const TSave savedefault = {
    DATAVER,                                      // (int) data format version
    0,                                            // (uint32_t) operation time in sec
    false,                                        // isChecked
    0.030f,                                       // gain_str;
    0.020f,                                       // gain_str_diff
    24.0f,                                        // gain_w_roll;
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},         // acc_offset
    {0.0f, 0.0f, 0.0f, 0.0175f, 0.9996f, 0.000f}, // acc_dir
    0.980f,                                       // str_diff_alph
    0,                                            // (int) steering angle neutral R= +deg
    60,                                           // (int) motor speed 0-99 (88.5RPM/4.0V, 57.9RPM/SPD=10)
    20,                                           // (int) stand for start
    0.0f,                                         // reserved
    0.012f,                                       // yaw‑rate feedback coefficient
    30,                                           // (int) steering_turn
    0xFFFFFFFF                                    // (uint32_t) CRC
};

//// R/C servo pulse width making
static const ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = SERVO_FREQ,
    .clk_cfg = LEDC_AUTO_CLK};

static ledc_channel_config_t svch_mot = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0, // モーター用
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_DRV, // dummy
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = 0};

static ledc_channel_config_t svch_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1, // ステアリング用
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR, // dummy
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = 0};

static ledc_channel_config_t svch_ex1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2, // サイドスタンド用
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_EX1, // dummy
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = 0};

///////////////////////////////////////////////////////////////////
/// in task web-server ///
void set_str_cmd(float angle, float step)
{
    str_step = step;
    str_cmd0 = angle;
}

// servo control task
// set_str_cmd(angle) -> str_cmd0 -> str_cmd1 -> str_cmd2 -> str_cal
static void str_easing()
{
    switch (runState)
    {
    case rsInner_Correct:
        str_cmd1 = str_cmd0 * AUTOCORRECTRATE;
        str_cmd2 = str_cmd1; // without easing
        break;

        // case rsInner_Stable:
        //     str_cmd1 = str_cmd0;
        //     str_cmd2 = str_cmd1; // without easing
        //     break;

    default:
        str_cmd1 = str_cmd0;
        if (str_step == 0.f)
        {
            str_cmd2 = str_cmd1;
        }
        else if (str_cmd1 - str_cmd2 > str_step)
        {
            str_cmd2 += str_step; // easing
        }
        else if (str_cmd1 - str_cmd2 < -str_step)
        {
            str_cmd2 -= str_step; // easing
        }
        else
        {
            str_cmd2 = str_cmd1;
        }
    }
}

/// servo control task
// angle: deg -90/+90 ang + = right turn
void str_pwm_out(float angle)
{
    str_out = angle;
    uint32_t pulsew = (angle + (float)saved.str0) * ANG2PULSE + 1500;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel,
                  USEC2LEDCDUTY(pulsew));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
}

/// in task web-server or any
void wait_str_angle()
{
    ESP_LOGI(TAG, "wait str");
    while (str_cmd2 != str_cmd1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "wait str end");
}

///////////////////////////////////////////////////////////////////
/// any task ///
// angle +-90deg , step 1/1500(us)
void set_ex1_angle(float angle, float step)
{
    ex1_step = step;
    ex1_cmd = angle * ANG2PULSE + 1500.0f;
}

/// in task web-server ///
void wait_ex1_angle()
{
    ESP_LOGI(TAG, "wait ex1");
    while (ex1_cmd != ex1_out)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "wait end");
}

/// in waitms() ///
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel,
                  USEC2LEDCDUTY((int)ex1_out));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);
}

/// Continuous rotation motor servo using for drive /////////////////////////
/// in task web-server ///
void wait_mot_duty()
{
    while (mot_cmd != mot_out)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// duty = -100.0(500us) / +100.0(2500us)
// static uint32_t stoptime;
// static bool timer_ON = false;
void set_mot_duty(float duty, float step)
{
    mot_cmd = duty;
    mot_step = step;
    // timer_ON = false;
}

/// in waitms() ///
static void do_mot_out()
{
    // if (timer_ON && stoptime <= millis)
    // {
    //     set_mot_duty(0, 1500);
    // }

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

    int mot_pw = (int)(mot_out * (1000.0f / 100.0f)) + 1500; // mot_cmd = +-100
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

//// STR servo PWM Lo->Hi edge trigger interrupt ////////////////////////////
TaskHandle_t xControlTaskHandle = NULL;
esp_timer_handle_t delay_timer;

static void ControlTask(void *pvParameters)
{
    void gyroServiceLoop(), do_str_cmd_calc();

    acc_offset = saved.acc_offset;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for interrupt

        gpio_set_level(IO_1, 1); // IR LED ON
        gyroServiceLoop();       // control.c
        put_data(&acc);
        do_ex1_out();
        do_mot_out();
        do_str_cmd_calc();
        str_easing();
        gpio_set_level(IO_1, 0);
        saved.acc_offset = acc_offset;
    }
}

static void IRAM_ATTR timer_callback(void* arg)
{
    xTaskNotifyGive(xControlTaskHandle);  // wakeup task
}

static void IRAM_ATTR gpio_str_isr_handler(void *arg)
{
    esp_timer_start_once(delay_timer, DELAY_TIME_USEC);  // in usec
}

void servo_init()
{
    extern void control_init();

    pyaw_coeff = &saved.yaw_coeff;

    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&svch_mot));
    ESP_ERROR_CHECK(ledc_channel_config(&svch_str));
    ESP_ERROR_CHECK(ledc_channel_config(&svch_ex1));

    control_init();

    str_pwm_out(0);
    set_mot_duty(0, 1000);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 1000);

    const esp_timer_create_args_t timer_args = { // hardware timer use
        .callback = timer_callback,
        .name = "high_res_delay"
    };
    esp_timer_create(&timer_args, &delay_timer);

    xTaskCreate(ControlTask, "ControlTask", 2048, NULL, configMAX_PRIORITIES - 1, &xControlTaskHandle);
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // positive edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_STR_IN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(GPIO_STR_IN, gpio_str_isr_handler, NULL);
    auto_disable();
}
