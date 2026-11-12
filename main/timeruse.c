#include "project.h"

// servo signals 1cycle for 4 stages (s0-s3) 1000us each
const ledc_timer_config_t servo_PWM_timer = {
    .speed_mode = LED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT, // 4096 steps for 100% duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 1000, // 250 Hz * 4 = 1000 Hz (1000 us per stage)
    .clk_cfg = LEDC_AUTO_CLK};

const ledc_channel_config_t svch_mot = {
    .speed_mode = LED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_DRV,
    .duty = PWM_DUTY_100, // Initial s0 duty is 100% (starts high).
    .hpoint = 0};

const ledc_channel_config_t svch_str = {
    .speed_mode = LED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_STR,
    .duty = PWM_DUTY_100, // Initial s0 duty is 100% (starts high).
    .hpoint = 0};

ledc_channel_config_t svch_ex1 = {
    .speed_mode = LED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = 255,    // dummy
    .duty = PWM_DUTY_0, // Initial s0 duty is 0% (starts low) to delay the side stand by one stage.
    .hpoint = 0};

// LED
const ledc_timer_config_t ledc_timer = {
    .speed_mode = LED_MODE,
    .timer_num = LED_TIMER,
    .duty_resolution = LED_RESOLUTION,
    .freq_hz = LED_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK};

const ledc_channel_config_t ledc_channel = {
    .gpio_num = LED_GPIO,
    .speed_mode = LED_MODE,
    .channel = LED_CHANNEL,
    .timer_sel = LED_TIMER,
    .duty = 0, // Initial brightness
    .hpoint = 0};
