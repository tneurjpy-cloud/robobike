/*
    RC bicycle with WiFi captive portal

    Copyright 2025.06.17 M.Tanaami
*/

#include "userdefine.h"

extern void showTasks();

static const char *TAG = "main";

////////////////////////////////////////////////////////////////////////
void app_main(void)
{
    ESP_LOGI(TAG, "Start ROBOBIKE system");

    void userdeviceinit();
    int maxcount = 0;
    uint32_t last1s;
    uint32_t last10s;

    esp_log_level_set("*", ESP_LOG_INFO);
    userdeviceinit();
    webserver_start();
    set_led_brightness(LEDLOW);
    set_str_cmd(0.0f, 100.0f);

    printf("PROGVER=%d DATAVER=%u SYSID=%s\n", PROGVER, DATAVER, SysID());
    showTasks();

    last1s = millis;
    last10s = millis;
    for (;;)
    { // what a kind of "serviceLoop()"
        waitms(100);
        // 転倒判定
        if (auto_en && (str_out >= (STRMAX - 1) || str_out <= -(STRMAX - 1)))
        {
            if (maxcount >= 2)
            {
                auto_disable();
                set_mot_duty(0.f, 2000.f);
                str_cmd0 = 0.f;
                waitms(500);
                set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 2);
                set_str_cmd(0.f, 100.f);
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

        // sleep 判定
        if ((mot_out == 0) && ((millis - userLastControlTime) > SLEEP_DURATION_MS))
        {
            deepSleep(0);
        }

        if (isNms(&last1s, 1000))
        {
            // ESP_LOGI(TAG,"gy.y=%7.3f", gy.y);
        }

        if (isNms(&last10s, 60000))
        {
            // showTasks();
        }
    }
}
