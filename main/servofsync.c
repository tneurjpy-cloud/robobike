/*//////////////////////////////////////////////////////////////////////////
    servofsync.c

サーボ制御用タイマーchart (5ステージ・5000usフレーム・全サーボ相乗り版)

    == : H level
    __ : L level or stop
    -- : count up running, waiting
    +  : start
    |  : fire interrupt

LEDC_CHANNEL_1, 0, 2 --> GPIO_STR, DRV, EX1 (s1 duty可変0-100%)
           s0           s1            s2          s3
____+============+======_______+____________+____________+=====
        1000us    500us  500us     1000us       1000us

GPIO_INT <-- GPIO_STR external interrupt 4ms interval
    cbPulseIn
____|____________________________________________________|_____
    start sync_timer

sync_timer(gptimer)
CB         cb0          cb1           cb2          cb3
____+------|------------|-------------|------------|_____+-----
    0     400us        1400us        2400us       3400us
    FIRSTWT  STGINTRVL    STGINTRVL    STGINTRVL

CallBackFunctions
cbStrIn: (GPIO external interrupt)
  FIRSTWTus後にcb0を起動するようsync_timerを開始

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
  str,motサーボのs0のduty=100%を書き出し
  sync_timerは停止

//////////////////////////////////////////////////////////////////////////*/

#include "userdefine.h"

static const char TAG[] = "servofsync";

//// R/C servo pulse width making
#define TIMER_RES_HZ 1000000 //
#define PWM_DUTY_0 0         //
#define PWM_DUTY_100 4096    // 12bit完全100%固定値
#define PWM_STAGE_LEN 1000   // us
#define PWM_MAXLEN 1990      // us
#define PWM_MINLEN 1010      // us
#define CB0DELAY 10          // us
#define FIRSTWT 400          // us
#define STG_INTRVL 1000      // us

/* 全サーボ共通：12bit高精度タイマーへ一本化 */
static const ledc_timer_config_t servo_timer_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT, // 4096 steps for 100% duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 250 * 4, // 250Hz * 4 = 1000Hz (1ステージ1000us)
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
    .gpio_num = 0,      // dummy
    .duty = PWM_DUTY_0, // ★スタンドは1ステージ遅らせるため、初期状態のs0は0%（L開始）
    .hpoint = 0};

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

// タスクが計算した std（ex1）のs2用可変Dutyを一時保持するバッファ変数
static volatile uint32_t duty_ex1_s2 = 0;

/////////////////////////////////////////////////////////////////////////////
// Only for PCB >= v06
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
    case cb0: // s0
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
        if (!stopServo)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_100);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);
        }
        next_alarm.alarm_count = FIRSTWT + STG_INTRVL;
        gptimer_set_alarm_action(timer, &next_alarm);
        res = true;
        sync_step++;
        break;

    case cb1: // s1
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
        if (!stopServo)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, duty_ex1_s2);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);
        }
        next_alarm.alarm_count = FIRSTWT + STG_INTRVL * 2;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb2: // s2
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);

        next_alarm.alarm_count = FIRSTWT + STG_INTRVL * 3;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        break;

    case cb3: // s3
        if (!stopServo)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_100);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel, PWM_DUTY_100);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_mot.channel);
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_ex1.channel);
        sync_step = cb0;
        gptimer_stop(timer); // 自走終了 外部割込み待ちへ
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

void servo_init_FSYNC()
{
    // サーボ信号検出ポート設定
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << GPIO_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INT, gpio_PulseIn_isr_handler, NULL);

    // サーボ信号生成タイマー設定
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    gptimer_new_timer(&timer_config, &sync_timer);
    gptimer_event_callbacks_t cbs = {
        .on_alarm = sync_timer_isr_cb,
    };
    gptimer_register_event_callbacks(sync_timer, &cbs, NULL);
    gptimer_enable(sync_timer);

    if (PCBver <= 5)
        svch_ex1.gpio_num = GPIO_NUM_9; // PCB v05 for stand servo;
    else
        svch_ex1.gpio_num = GPIO_NUM_6; // PCB v06 or later;

    // PWMタイマー設定
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