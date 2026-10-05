/*
    ROBOBIKE with WiFi captive portal
*/

#include "userdefine.h"

static const char *TAG = "main";

////////////////////////////////////////////////////////////////////////
void check_sleep_indicator(void)
{
    esp_sleep_source_t cause = esp_sleep_get_wakeup_causes();

    if (cause != ESP_SLEEP_WAKEUP_ALL)
    { // 450ms to do reset to shutdown
        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

        // 一瞬ピカッと光らせる（例: 30msだけ点灯）
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(30));
        gpio_set_level(LED_GPIO, 0);

        // 再び10秒のタイマー設定をしてディープスリープへ戻る
        esp_sleep_enable_timer_wakeup(SLEEPINTERVAL * 1000ULL); // usec
        esp_deep_sleep_start();
    }
}

void deviceCheck()
{
    // str
    set_str_cmd(15.0f, 0.0f);
    waitms(250);
    set_str_cmd(0.0f, 0.0f);
    waitms(250);

    // mot
    set_mot_duty(10.0f, 0.0f);
    waitms(150);
    set_mot_duty(0.0f, 0.0f);
    waitms(250);

    // ex1
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT + 10.0f, 0.0f);
    waitms(250);
    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.05f);
}

void app_main(void)
{
    int maxcount = 0;
    uint32_t lastdone;

    check_sleep_indicator();

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Start ROBOBIKE system");

    userdeviceinit();
    if (IMU_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        while (true)
        {
            set_led_brightness(LEDHIGH);
            waitms(125);
            set_led_brightness(LEDOFF);
            waitms(125);
        }
    }

    servo_init();
    webserver_start();

    deviceCheck();

    set_led_brightness(LEDLOW);
    // set_str_cmd(0.0f, 100.0f);
    printf("PROGVER=%d DATAVER=%u SYSID=%s\n", PROGVER, DATAVER, SysID());
    ESP_LOGI(TAG, "OpTime=%lusec\n", saved.op_time_s);
    showTasks();

    lastdone = millis();
    for (;;)
    {
        waitms(100);

        if (auto_en && (str_out >= (STRMAX - 2) || str_out <= -(STRMAX - 2)))
        { // 転倒判定
            extern uint32_t duty_str_prev;
            ESP_LOGI(TAG, "Str duty_s1=%lu", duty_str_prev);
            if (maxcount >= 5)
            {
                auto_disable();
                set_mot_duty(0.f, 0.f);
                waitms(100);
                set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.1f);
                set_str_cmd(0.f, 0.5f);
                maxcount = 0;
                set_led_brightness(LEDLOW);
            }
            else
            {
                maxcount++;
            }
        }
        else
        {
            maxcount = 0;
        }

        if (isNms(&lastdone, 1000))
        {
            if ((mot_out == 0.0f) && ((millis() - userLastControlTime) >= SLEEP_DURATION_MS))
            {
                deepSleep(SLEEPINTERVAL);
            }
        }
    }
}
