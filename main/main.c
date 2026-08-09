/*
    ROBOBIKE with WiFi captive portal
*/

#include "userdefine.h"

static const char *TAG = "main";

////////////////////////////////////////////////////////////////////////
void app_main(void)
{
    int maxcount = 0;
    uint32_t lastdone;

    ESP_LOGI(TAG, "Start ROBOBIKE system");
    esp_log_level_set("*", ESP_LOG_INFO);
    userdeviceinit();
    IMU_init();
    servo_init();
    webserver_start();

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
            if (maxcount >= 2)
            {
                auto_disable();
                set_mot_duty(0.f, 0.f);
                waitms(100);
                set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.1f);
                set_str_cmd(0.f, 0.5f);
                maxcount = 0;
                set_led_brightness(LEDLOW);
                savenvs();
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
            // extern bool IO2;
            // ESP_LOGI(TAG, "IO2=%d", IO2);
            if ((mot_out == 0.0f) && ((millis() - userLastControlTime) >= SLEEP_DURATION_MS))
            {
                deepSleep(0);
            }
        }
    }
}
