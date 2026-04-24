#include "userdefine.h"

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

////////////////////////////////////////////////////////
typedef struct
{
    uint32_t time;
    Tvector6d acc;
    float drv;
    float str;
    float std;
} Tlogvector;

#define RING_BUF_SIZE (SAMPLE_RATE_HZ * 3)
Tlogvector ring_buffer[RING_BUF_SIZE];
static int index_w = 0;
static int index_r = 0; // 最後に読み出した位置

////////////////////////////////////////////////////////
/// called in 5msec loop
void put_data(Tvector6d *new_data)
{
    Tlogvector *p;

    p = &ring_buffer[index_w];
    p->time = millis;
    p->acc = *new_data;
    p->drv = mot_out;
    p->str = str_out;
    p->std = ex1_out;
    p->drv = mot_out;
    p->str = str_out;
    p->std = ex1_out;
    index_w = (index_w + 1) % RING_BUF_SIZE;

    // もし未取得データがn秒分を超えたら、index_rを強制的に進める（古いデータを捨てる）
    if (index_w == index_r)
    {
        index_r = (index_r + 1) % RING_BUF_SIZE;
    }
}

const char *get_data()
{
    static char buf[4096];
    char item_buf[96];

    /*  for java script
    const MON_CSV_KEYS = [
        "HEADER",   // 0: 'a'
        "TIME_MS",  // 1
        "SV_DRV",   // 2
        "SV_STR",   // 3
        "SV_STD",   // 4
        "GY_ROLL",  // 5
        "GY_YAW",   // 6
        "GY_PITCH", // 7
    ];
    */
    buf[0] = '\0';
    while (index_r != index_w)
    {
        Tlogvector *p = &ring_buffer[index_r];
        Tvector6d *pa = &p->acc;
        int len = snprintf(item_buf, sizeof(item_buf),
                           "a,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                           p->time, p->drv, p->str, p->std, pa->ROLL_A, pa->YAW_A, pa->PITCH_A);
        if (strlen(buf) + len + 1 > sizeof(buf))
        {
            break;
        }
        strcat(buf, item_buf);
        index_r = (index_r + 1) % RING_BUF_SIZE;
    }
    return buf;
}
