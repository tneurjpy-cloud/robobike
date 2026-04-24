#include "userdefine.h"
#include "web_common.h"

static const char *TAG = "web_api";

char *mkcsv()
{
    static char rescsv[128];
    memset(rescsv, '\0', sizeof(rescsv));

    float str_cmd_rate = (float)str_cmd0 / (float)saved.str_turn * STR_SLIDER_MAX;

    /*  for java script
    const CSV_KEYS = [
        "HEADER",       // 0: 'b'
        "PROG_VER",     // 1
        "DATA_VER",     // 2
        "STATUS",       // 3
        "STR0",         // 4
        "MOT_SPD",      // 5
        "GAIN_STR",     // 6
        "GAIN_W_ROLL",  // 7
        "GAIN_DIFF",    // 8
        "ANG_STD_NUT",  // 9
        "STR_TURN",     // 10
        "YAW_COEFF",    // 11
        "AUTO_CIRCLING",// 12
        "STR_CMD_RATE"  // 13
    ];
    */
    snprintf(rescsv, sizeof(rescsv),
             "b,%d,%d,%s,%d,%d,%.3f,%.3f,%.3f,%d,%d,%.3f,%d,%.3f",
             PROGVER,
             DATAVER,
             "OK",
             saved.str0,
             saved.mot_spd,
             saved.gain_str,
             saved.gain_w_roll,
             saved.gain_str_diff,
             saved.ang_std_nut,
             saved.str_turn,
             saved.yaw_coeff,
             autoCircling,
             str_cmd_rate);

    ESP_LOGI(TAG, "length=%d\ncsv=%s", strlen(rescsv), rescsv);

    return rescsv;
}

