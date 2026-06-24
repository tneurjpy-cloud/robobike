/*//////////////////////////////////////////////////////////////////////////
    servo.c

サーボ制御用タイマーchart (4ステージ・4000usフレーム・全サーボ完全相乗り版)

    == : H level
    __ : L level or stop
    -- : count up running, waiting
    +  : start
    |  : fire interrupt

LEDC_CHANNEL_1, 0, 2 --> GPIO_STR, DRV, EX1 (s1 duty可変0-100%)
           s0           s1            s2          s3
____+============+======_______+____________+____________+=====
        1000us    500us  500us     1000us       1000us

GPIO_STR_IN <-- GPIO_STR external interrupt 4ms interval
    cbPulseIn
____|____________________________________________________|_____
    start sync_timer

sync_timer(gptimer)
CB         cb0          cb1           cb2          cb3
____+------|------------|-------------|------------|_____+-----
    0     550us        1550us        2550us       3550us
    FIRSTWT  STGINTRVL    STGINTRVL    STGINTRVL

CallBackFunctions
cbStrIn: (GPIO external interrupt)
  sync_timerを、450us後割込み待ちでスタート

cb0: (gptimer countup interrupt)
  ControlTask起床
  IMUのデータ取り込みを行い、全サーボの計算、
  str,mot s1のduty=0-50% を書き出し
  stdサーボのs0のduty=100%を書き出し
  1000us後のtimer割り込みをスタート

cb1: (gptimer countup interrupt)
  std s1のduty=0-50% を書き出し
  str,motサーボのs2のduty=0%を書き出し
  1000us後のtimer割り込みをスタート

cb2: (gptimer countup interrupt)
  stdサーボのs2のduty=0%を書き出し
  1000us後のtimer割り込みをスタート

cb3: (gptimer countup interrupt)
  sync_timerは停止
  str,motサーボのs0のduty=100%を書き出し

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
#define TIMER_RES_HZ 1000000 //
#define PWM_DUTY_0 0         //
#define PWM_DUTY_100 4096    // 12bit完全100%固定値
#define PWM_STAGE_LEN 1000   // us
#define PWM_MAXLEN 1990      // us
#define PWM_MINLEN 1010      // us
#define CB0DELAY 10          // us
#define FIRSTWT 450          // us
#define STG_INTRVL 1000      // us

/* 全サーボ共通：12bit高精度タイマーへ一本化 */
static const ledc_timer_config_t servo_timer_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT, // 4096 steps for 100% duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = SV_FRQ * 4, // 250Hz * 4 = 1000Hz (1ステージ1000us)
    .clk_cfg = LEDC_AUTO_CLK};

static ledc_channel_config_t svch_mot = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_DRV,
    .duty = PWM_DUTY_100, // 初期状態のs0は100%（H開始）
    .hpoint = 0};

static ledc_channel_config_t svch_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR,
    .duty = PWM_DUTY_100, // 初期状態のs0は100%（H開始）
    .hpoint = 0};

static ledc_channel_config_t svch_ex1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_EX1,
    .duty = PWM_DUTY_0, // ★スタンドは1ステージ遅らせるため、初期状態のs0は0%（L開始）
    .hpoint = 0};

// gpTimer for synchronization
static gptimer_handle_t sync_timer;
static TaskHandle_t xControlTaskHandle;
typedef enum
{
    cb0 = 0,
    cb1,
    cb2,
    cb3,
} TSyncCBStep;
static volatile TSyncCBStep sync_step;

// タスクが計算した std（ex1）のs2用可変Dutyを一時保持するバッファ変数
static volatile uint32_t duty_ex1_s2 = 0;

// ★遅刻時の2000usパルス化を防ぐための、前回計算値保持用変数
static uint32_t duty_str_prev = 2048; // 初期値ニュートラル(1500us相当)
static uint32_t duty_mot_prev = 2048; // 初期値ニュートラル(1500us相当)

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

    int32_t pulsew = (int32_t)(ex1_out * ANG2PULSE) + 1500;

    if (pulsew < PWM_MINLEN)
        pulsew = PWM_MINLEN;
    if (pulsew > PWM_MAXLEN)
        pulsew = PWM_MAXLEN;

    uint32_t pulse_s1 = pulsew - PWM_STAGE_LEN;

    // ★直接レジスタは叩かず、バッファに保存（cb2のタイミングで安全に仕込むため）
    duty_ex1_s2 = (pulse_s1 * PWM_DUTY_100) / PWM_STAGE_LEN;
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

    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, duty_s1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
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
    switch (runState)
    {
    case rsInner_Correct:
        str_cmd1 = str_cmd0 * AUTOCORRECTRATE;
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
    int32_t pulsew = (int32_t)((angle + (float)saved.str0) * ANG2PULSE) + 1500;

    if (pulsew < PWM_MINLEN)
        pulsew = PWM_MINLEN;
    if (pulsew > PWM_MAXLEN)
        pulsew = PWM_MAXLEN;

    uint32_t pulse_s1 = pulsew - PWM_STAGE_LEN;
    uint32_t duty_s1 = (pulse_s1 * PWM_DUTY_100) / PWM_STAGE_LEN;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, duty_s1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
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

///////////////////////////////////////////////////////////////////
void gyroServiceLoop()
{
    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff;
    float w_roll_cmd;

    IMU_startRead();
    if (auto_en)
    {
        str_dev = str_target - str_out;
        str_dev_diff = (str_dev - last_str_dev) * SV_FRQ;
        str_diff_lps = (1.0f - saved.str_diff_alph) * str_diff_lps + saved.str_diff_alph * str_dev_diff;
        w_roll_cmd = str_dev * saved.gain_str + str_diff_lps * saved.gain_str_diff;
        w_roll_dev = w_roll_cmd - IMU_roll();
        str_out -= w_roll_dev * saved.gain_w_roll * (1.0f / SV_FRQ);
        last_str_dev = str_dev;
        chklimit(&str_out, STRMAX);
        str_pwm_out(str_out);
    }
    else
    {
        last_str_dev = 0.0f;
        str_diff_lps = 0.0f;
        str_out = 0.0f;
        str_pwm_out(str_target);
        if (IMU_getZero())
            saved.acc_offset = acc_offset;
    }
}

static void ControlTask(void *pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for servo pulse timing

        gpio_set_level(IO_1, 1); // IR LED ON

        // ★【追加】I2Cで待たされる前に、前回値を保険として先出ししておく
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, duty_str_prev);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, duty_mot_prev);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);

        gyroServiceLoop();       // 最速でI2C読み込み（間に合えば最新値に上書きラッチされる）

        gpio_set_level(IO_1, 0); // IR LED OFF
        gpio_set_level(IO_1, 1); // IR LED

        do_mot_out();
        do_str_cmd_calc();       // auto circling calc
        do_ex1_out();
        str_easing();
        put_control_data();

        gpio_set_level(IO_1, 0); // IR LED OFF
    }
}

// 【主軸 gptimer 割り込みハンドラ】1000us等間隔の数珠繋ぎステートマシン
static bool IRAM_ATTR sync_timer_isr_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gptimer_alarm_config_t next_alarm = {0};
    next_alarm.flags.auto_reload_on_alarm = false;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool res = false;

    switch (sync_step)
    {
    case cb0:
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);

        next_alarm.alarm_count = FIRSTWT + STG_INTRVL;
        gptimer_set_alarm_action(timer, &next_alarm);
        res = true;
        sync_step++;
        break;

    case cb1:

        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, duty_ex1_s2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);

        next_alarm.alarm_count = FIRSTWT + STG_INTRVL * 2;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb2:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);

        next_alarm.alarm_count = FIRSTWT + STG_INTRVL * 3;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb3:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, PWM_DUTY_100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);

        gptimer_stop(timer); // 自走終了 外部割込み待ちへ
        sync_step = cb0;
        break;
    }

    return res;
}

// PulseIn: サーボ信号検出外部割り込みハンドラ s0開始=GPIO_STRの立ち上がりを検知
static void IRAM_ATTR gpio_PulseIn_isr_handler(void *arg)
{
    static gptimer_alarm_config_t first_alarm = {
        .alarm_count = FIRSTWT,
        .flags.auto_reload_on_alarm = false,
    };

    // 外部割込み待ちフェーズの時だけ反応
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

    // 制御計算タスク生成
    control_init();
    xTaskCreate(ControlTask, "ControlTask", 2048, NULL, configMAX_PRIORITIES - 1, &xControlTaskHandle);

    // サーボ信号検出ポート設定
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << GPIO_STR_IN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_STR_IN, gpio_PulseIn_isr_handler, NULL);

    // サーボ信号生成タイマー設定
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    gptimer_new_timer(&timer_config, &sync_timer);
    sync_step = cb0;
    gptimer_event_callbacks_t cbs = {
        .on_alarm = sync_timer_isr_cb,
    };
    gptimer_register_event_callbacks(sync_timer, &cbs, NULL);
    gptimer_enable(sync_timer);

    // サーボ信号PWM設定
    ledc_timer_config(&servo_timer_str);
    ledc_channel_config(&svch_mot);
    ledc_channel_config(&svch_str);
    ledc_channel_config(&svch_ex1);

    // servo initial value
    set_mot_duty(0.0f, 0.0f);
    set_str_cmd(0.0f, 0.0f);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.0f);

    auto_disable();
    ESP_LOGI(TAG, "All 3 servos integrated into 4-STAGE INTERLEAVED SYNC mode.");
}
