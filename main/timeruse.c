#include "userdefine.h"

// servo signals
const ledc_timer_config_t servo_PWM_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT, // 4096 steps for 100% duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 1000, // 250Hz * 4 = 1000Hz (1ステージ1000us)
    .clk_cfg = LEDC_AUTO_CLK};

ledc_channel_config_t svch_mot = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_DRV,
    .duty = PWM_DUTY_100, // 初期状態のs0は100%（H開始）
    .hpoint = 0};

ledc_channel_config_t svch_str = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR,
    .duty = PWM_DUTY_100, // 初期状態のs0は100%（H開始）
    .hpoint = 0};

ledc_channel_config_t svch_ex1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = 0,      // dummy
    .duty = PWM_DUTY_0, // ★スタンドは1ステージ遅らせるため、初期状態のs0は0%（L開始）
    .hpoint = 0};

// LED
ledc_timer_config_t ledc_timer = {
    .speed_mode = LED_MODE,
    .timer_num = LED_TIMER,
    .duty_resolution = LED_RESOLUTION,
    .freq_hz = LED_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK};

ledc_channel_config_t ledc_channel = {
    .gpio_num = LED_GPIO,
    .speed_mode = LED_MODE,
    .channel = LED_CHANNEL,
    .timer_sel = LED_TIMER,
    .duty = 0, // 初期輝度
    .hpoint = 0};
