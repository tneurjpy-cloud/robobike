/*//////////////////////////////////////////////////////////////////////////
    servo.c

サーボ制御用タイマーchart (4ステージ・4000usフレーム)

    == : H level
    __ : L level or stop
    +  : start
    |  : fire interrupt
    -- : count up running, waiting

INT of GPIO_STR_IN <-- GPIO_STR
      cbStrIn
_______|____________________________________________________|_____
       ^ start sync_timer 100us

LEDC_CHANNEL_1(ledc_timer) --> GPIO_STR (s1可変を最大50%に制限時)
            s0            s1            s2          s3
_______+============+======_______+____________+____________+=====
           1000us    500us  500us     1000us

sync_timer(gptimer)
          cb0   cb1           cb2                       cb3  CallBack
__________+-----|+------------|+------------------------|___+-----
           650us    1000us             2000us

CallBackFunctions
cbStrIn: (GPIO external interrupt)
  100us後にcb0をたたくようsynctimerを起動
  時間はオシロで確認後決定

cb0: (gptimer countup interrupt)
 650us後のsync_timer割り込みをスタート
 IMUのリセットを行う

cb1: (gptimer countup interrupt)
 1000us後のtimer割り込みをスタート
 ControlTask起床
  IMUのデータ取り込みを行い、ステアサーボの計算、
    s1のduty=0-50% 書き出し
  その他タイミングが厳しくないサーボの計算を行い、LEDCセット

cb2: (gptimer countup interrupt)
 2000us後のtimer割り込みをスタート
  s2のduty=0%を書き出し

cb3: (gptimer countup interrupt)
  sync_timerは停止
  s0のduty=100%を書き出し

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
#define USEC2LEDCDUTY(x) (((x) * 16384) / (1000000 / SV_FRQ)) // 2^14 for 100% duty
#define TIMER_RES_HZ 1000000                                  //
#define PWM_DUTY_0 0
#define PWM_DUTY_100 4096
#define PWM_STAGE_LEN 1000 // us
#define PWM_MAXLEN 2000
#define PWM_MINLEN 1001
#define CB0DELAY 100       // us

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
    .hpoint = USEC2LEDCDUTY(500)};

static ledc_channel_config_t svch_ex1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_EX1,
    .duty = SERVO_NEUTRAL_DUTY,
    .hpoint = USEC2LEDCDUTY(1000)};

///// ステアリングサーボ用LEDCタイマ設定
static const ledc_timer_config_t servo_timer_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT, // 4096 steps for 100% duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = SV_FRQ * 4, // 4ステージのため SV_FRQ(250) * 4 = 1000Hz
    .clk_cfg = LEDC_AUTO_CLK};

static ledc_channel_config_t svch_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR,
    .duty = PWM_DUTY_100,
    .hpoint = 0};

// gpTimer for synchronization
static gptimer_handle_t sync_timer;
static TaskHandle_t xControlTaskHandle;
typedef enum
{
    ss0 = 0,
    ss1,
    ss2,
    ss3,
} TSyncStep;
static volatile TSyncStep sync_step = ss0;

///////////////////////////////////////////////////////////////////
/// any task ///
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
/// in task web-server ///
void set_str_cmd(float angle, float step)
{
    str_step = step;
    str_cmd0 = angle;
}

/// servo control task ///
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

    // 【可変範囲制限】パルス幅を 1200us 〜 1800us に厳密に制限
    if (pulsew < PWM_MINLEN)
        pulsew = PWM_MINLEN;
    if (pulsew > PWM_MAXLEN)
        pulsew = PWM_MAXLEN;

    uint32_t pulse_s1 = pulsew - PWM_STAGE_LEN;
    uint32_t duty_s1 = (pulse_s1 * PWM_DUTY_100) / PWM_STAGE_LEN;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, duty_s1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
}

/// task web-server or any
void wait_str_angle()
{
    ESP_LOGI(TAG, "wait str");
    while (str_target != str_cmd1)
    {
        waitTaskms(10);
    }
    ESP_LOGI(TAG, "wait str end");
}

//////////////////////////////////////////////////////////////////////////////
// task ControlTask
void gyroServiceLoop()
{
    static float last_str_dev = 0.0f;
    static float str_diff_lps = 0.0f;
    float w_roll_dev, str_dev, str_dev_diff; // 偏差
    float w_roll_cmd;

    IMU_startRead();                                          // Start I2C read of IMU data, will be available in 100us
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
        str_pwm_out(str_out);                                 // +: right steer
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

///////////////////////////////////////////////////////////////////////////////
// ControlTask
static void ControlTask(void *pvParameters)
{
    for (;;)
    { // Wait for Notify from sync_timer callback (cb1から起床信号を受ける)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(IO_1, 1); // IR LED ON
        gyroServiceLoop();       // Read IMU, calc control, update servo outputs (内部でs1のdutyを書き出し)
        gpio_set_level(IO_1, 0); // IR LED OFF
        put_control_data();      // Send control data to web-server task
        do_ex1_out();            // Side Stand calc.
        do_mot_out();            // Motor drive calc.
        do_str_cmd_calc();       // Area detection calc.
        str_easing();            // Easing for steering command
    }
}

// 【主軸 gptimer 割り込みハンドラ】ロングジャンプを駆使し、cb3のままで4.8msフレームを支配
static bool IRAM_ATTR sync_timer_isr_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gptimer_alarm_config_t next_alarm = {0};
    next_alarm.flags.auto_reload_on_alarm = false;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool res = false;

    switch (sync_step)
    {
    case ss0: // cb0: s0開始から100us経過時点
        next_alarm.alarm_count = CB0DELAY + ICM426XX_RESETWAIT;
        gptimer_set_alarm_action(timer, &next_alarm);
        gpio_set_level(IO_1, 1);
        IMU_ResetDigitalPath();
        gpio_set_level(IO_1, 0);
        sync_step++;
        break;

    case ss1: // cb1: s0開始から750us経過時点 (s1ラッチの450us前)
        next_alarm.alarm_count = CB0DELAY + ICM426XX_RESETWAIT + PWM_STAGE_LEN;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
        res = true;
        break;

    case ss2: // cb2: s1開始から750us経過時点 (s2ラッチの450us前)
        next_alarm.alarm_count = CB0DELAY + ICM426XX_RESETWAIT + PWM_STAGE_LEN * 3;
        gptimer_set_alarm_action(timer, &next_alarm);
        sync_step++;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        break;

    case ss3: // cb3: s0ラッチの450us前)
        gptimer_stop(timer);
        sync_step = ss0;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, svch_str.channel, PWM_DUTY_100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, svch_str.channel);
        break;
    }

    return res;
}

// 【cbStrIn: GPIO外部割り込みハンドラ】GPIO_STRの立ち上がりを検知
static void IRAM_ATTR gpio_srv_in_isr_handler(void *arg)
{
    static gptimer_alarm_config_t first_alarm = {
        .alarm_count = CB0DELAY, // 100us後にcb0を発火
        .flags.auto_reload_on_alarm = false,
    };

    if (sync_step != ss0) {
        return;
    }
    gptimer_set_raw_count(sync_timer, 0);
    gptimer_set_alarm_action(sync_timer, &first_alarm);
    gptimer_start(sync_timer);

    sync_step = 0;
}

////////////////////////////////////////////////////////////////////////////
/// initialization of servo control and Master Sync ///
void servo_init()
{
    pyaw_coeff = &saved.yaw_coeff;

    // LEDC setup for servo control
    ledc_timer_config(&servo_timer);
    ledc_timer_config(&servo_timer_str); // ステアリング専用12bitタイマー
    ledc_channel_config(&svch_mot);
    ledc_channel_config(&svch_str); // ステアリング用チャンネル有効化
    ledc_channel_config(&svch_ex1);

    control_init();
    xTaskCreate(ControlTask, "ControlTask", 2048, NULL, configMAX_PRIORITIES - 1, &xControlTaskHandle);

    // 1. Initialize GPIO_STR_IN for External Interrupt
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,        // 立ち上がりエッジで割り込み
        .pin_bit_mask = (1ULL << GPIO_STR_IN), // 対象のGPIOピン
        .mode = GPIO_MODE_INPUT,               // 入力モード
        .pull_down_en = GPIO_PULLDOWN_DISABLE, //
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 外部配線直結のためプルアップ/ダウンは無し
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_STR_IN, gpio_srv_in_isr_handler, NULL);

    // 2. Initialize GPTIMER as a One-Shot Delay Timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ, // 1MHz (1us単位)
    };
    gptimer_new_timer(&timer_config, &sync_timer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = sync_timer_isr_cb, // 4ステージ化された変則インターバルハンドラ
    };
    gptimer_register_event_callbacks(sync_timer, &cbs, NULL);
    gptimer_enable(sync_timer);

    // Set initial servo positions and disable outputs
    auto_disable();
    str_pwm_out(0);
    set_mot_duty(0, 0.0f);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.0f);
    ESP_LOGI(TAG, "Servo initialized with EXTERNAL HARDWARE 4-STAGE SYNC mode.");
}