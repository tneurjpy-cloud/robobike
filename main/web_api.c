#include "userdefine.h"

static const char *TAG = "web_api";

// 名前変換関数を定義
const char *cmdID_to_str(TcmdID id)
{
    switch (id)
    {
#define AS_STRING(name) \
    case name:          \
        return #name;
        COMMAND_LIST(AS_STRING)
#undef AS_STRING
    default:
        return "unknown";
    }
}

char *get_edit_data()
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
        "STR_CMD_RATE", // 13
        "STR_CMD_SPD"   // 14
    ];
    */
    snprintf(rescsv, sizeof(rescsv), "%c,%d,%d,%s,%d,%d,%.3f,%.3f,%.3f,%d,%d,%.3f,%d,%.3f,%d",
             'b',
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
             str_cmd_rate,
             saved.str_cmd_speed);

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

Tlogvector ring_buffer[RING_BUF_SIZE];
static int index_w = 0;
static int index_r = 0; // 最後に読み出した位置

////////////////////////////////////////////////////////
/// called in Control task for data logging
void put_control_data()
{
    Tlogvector *p;

    p = &ring_buffer[index_w];
    p->time = millis();
    p->acc = acc;
    p->drv = mot_out;
    p->str = str_out;
    p->std = ex1_out;
    index_w = (index_w + 1) % RING_BUF_SIZE;

    if (index_w == index_r) // Over flow,
    {
        index_r = (index_r + 1) % RING_BUF_SIZE;
    }
}

/*
a,3715500,-100.000,-90.000,-90.000,-111.267,-129.328,-134.046
64bytes/line * 200Hz*1sec=12800bytes
*/
// call this in web server task to get data for monitor
char *get_control_data()
{
    static char buf[CTL_DATA_BUFSIZE];

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
        char item_buf[96];
        Tlogvector *p = &ring_buffer[index_r];
        Tvector6d *pa = &p->acc;
        int len = snprintf(item_buf, sizeof(item_buf),
                           "%c,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                           'a',
                           p->time,
                           p->drv,
                           p->str,
                           p->std,
                           GY_ROLL_P(pa),
                           GY_YAW_P(pa),
                           GY_PITCH_P(pa));
        if (strlen(buf) + len + 1 > sizeof(buf)) // check buf length
        {
            break;
        }
        strcat(buf, item_buf);
        index_r = (index_r + 1) % RING_BUF_SIZE;
    }
    return buf;
}

///////////////////////////////////////////////////////////////////
// TASK of Control commands of long sequences
///////////////////////////////////////////////////////////////////
static TaskHandle_t xcmdProc_TaskHandle = NULL;
static QueueHandle_t control_queue = NULL;
static SemaphoreHandle_t xMutex = NULL;
static EventGroupHandle_t xCommandEventGroup = NULL;
#define CMD_F_BIT (0b00000001) // bit flag for now command was bt_F

// cmdProcTask ////////////////////////////////////////////////////
static void cmdProcTask(void *pvParameters)
{
    extern volatile bool PWM_phase_sync;

    for (;;)
    {
        TcmdID id; // Do not use msg.req.

        if (xQueueReceive(control_queue, &id, portMAX_DELAY) == pdPASS)
        { // do commands
            xSemaphoreTake(xMutex, pdMS_TO_TICKS(100));
            ESP_LOGI(TAG, "Task command= %s", cmdID_to_str(id));
            switch (id)
            {
            case stp_all:
                auto_disable();
                set_mot_duty(0.0f, SERVO_NEUTRAL_DUTY);
                set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, SERVO_NEUTRAL_DUTY);
                set_str_cmd(0, SERVO_NEUTRAL_DUTY);
                savenvs(); // save NVS flash memory
                break;

            case bt_F:
                xEventGroupSetBits(xCommandEventGroup, CMD_F_BIT);
                if (mot_out != 0.0f) // 走行中
                {
                    set_str_cmd(0.0f, saved.str_cmd_speed * 0.5f / (float)SV_FRQ); // ゆっくりと戻す
                }
                else // start on stop
                {    // down the stand slowly, motor on, up the stand
                    auto_disable();
                    set_led_brightness(LEDHIGH);
                    PWM_phase_sync = true; // 位相が時折狂うため、発進時にリセットをかける
                    set_str_cmd(0.0f, 0.0f);
                    set_ex1_angle((float)((STD_STD_NUT + saved.ang_std_nut) + saved.ang_std_nut * 3) / 4.0f, 0.1f);
                    wait_ex1_angle();
                    set_ex1_angle(saved.ang_std_nut, 0.02f);
                    wait_ex1_angle();
                    set_mot_duty(saved.mot_spd, 50.f);
                    auto_enable();
                    set_ex1_angle(STD_RUN, 1.f);
                } // 指定したビットを落とす
                xEventGroupClearBits(xCommandEventGroup, CMD_F_BIT);
                break;

            case bt_S:
                bool s = false;
                if (mot_out != 0.0f)
                {
                    s = true;
                }
                if (mot_out > 0)
                {
                    if (str_target != 0.f)
                    {
                        set_str_cmd(0.0f, 60.0f / (float)SV_FRQ);
                        wait_str_angle();
                        waitTaskms(100);
                    }
                    set_str_cmd(-STR_STOP, 1.0f);                         // 左舵
                    set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, 0.0f); // スタンドを先に出す
                    waitTaskms(400);
                    auto_disable();
                    set_str_cmd(STR_STOP, 0.0f);
                    set_mot_duty(0.0f, 4.0f);
                    waitTaskms(400);
                    set_led_brightness(LEDLOW);
                }
                else
                {
                    set_mot_duty(0.0f, 0.0f);
                    set_led_brightness(LEDLOW);
                }
                set_str_cmd(0.0f, 0.0f);

                if (s)
                {
                    waitTaskms(50);
                    savenvs();
                }
                break;

            case bt_Std_nutAuto:
                set_mot_duty(0.0f, 0.0f);
                set_ex1_angle(-10.0f, 0.1f);
                while (ex1_out > ex1_cmd)
                {
                    ESP_LOGI(TAG, "az=%f", LATERAL_G);
                    waitTaskms(20);
                    if (ex1_out <= ex1_cmd)
                    {
                        break;
                    }
                    else if (LATERAL_G <= 0.5f)
                    {
                        waitTaskms(40);
                        if (LATERAL_G <= 0.5f)
                        {
                            break;
                        }
                    }
                }

                if (saved.ang_std_nut < EX1MAX)
                    saved.ang_std_nut += 1;

                set_ex1_angle(saved.ang_std_nut, 1);
                break;

            default:
                break;
            }
            xSemaphoreGive(xMutex);
        }
    }
}

//// httpd task /////////////////////////////////////////////////////////////////////////
esp_err_t put_command(control_msg_t *msg)
{
    esp_err_t res = pdPASS;

    if (control_queue == NULL) // 排他制御のためのオブジェクトを準備
    {
        control_queue = xQueueCreate(1, sizeof(control_msg_t));
        xMutex = xSemaphoreCreateMutex();
        xCommandEventGroup = xEventGroupCreate();

        // 自身の優先度を取得
        UBaseType_t currentPriority = uxTaskPriorityGet(NULL);
        UBaseType_t newPriority = (currentPriority > 0) ? (currentPriority - 1) : 0;
        xTaskCreate(cmdProcTask, "cmdProcTask", 3072, NULL, newPriority, &xcmdProc_TaskHandle);
        ESP_LOGI(TAG, "cmdProcTask created.");
    }

    if (xSemaphoreTake(xMutex, 0) != pdTRUE) // Mutexを取れなかった場合
    {                                        // 長時間コマンド実行中である
        if ((xEventGroupGetBits(xCommandEventGroup) & CMD_F_BIT) && (msg->id == bt_L || msg->id == bt_R))
        { // 発進シーケンス中の操舵は無視せず待つ
            xSemaphoreTake(xMutex, portMAX_DELAY);
        }
        else
        { // ほかの場合は何もせず帰る
            return res;
        }
    }
    // ここまで来たらMutexを取れている
    xSemaphoreGive(xMutex);
    ESP_LOGI(TAG, "put cmd= %s", cmdID_to_str(msg->id));

    switch (msg->id)
    { // 即座に帰るべきコマンドはこの関数内で処理
    case unknown:
        break;

    case only_data:
        break;

    case bt_L:
        set_str_cmd(-saved.str_turn, saved.str_cmd_speed / (float)SV_FRQ);
        break;

    case bt_R:
        set_str_cmd(saved.str_turn, saved.str_cmd_speed / (float)SV_FRQ);
        break;

    case bt_Str_S: // Control by sliding bar
        char query_str[64];
        char value_str[64];

        res = httpd_req_get_url_query_str(msg->req, query_str, sizeof(query_str));
        res = httpd_query_key_value(query_str, "value", value_str, sizeof(value_str));
        float value = atof(value_str);
        set_str_cmd(value * saved.str_turn / STR_SLIDER_MAX, saved.str_cmd_speed / (float)SV_FRQ);
        break;

    case bt_BK:
        if (mot_out == 0) // 停止中
        {
            set_mot_duty(MOT_SPEED_BACK, (40.0f / SV_FRQ));
            set_led_brightness(LEDHIGH);
        }
        else if (mot_out < 0)
        {
            set_mot_duty(0.0f, (40.0f / SV_FRQ));
            set_led_brightness(LEDLOW);
        }
        break;

    case bt_A_Up_0:
        if (saved.mot_spd < MOTMAX)
            saved.mot_spd++;
        if (mot_out > 0) // 走行中
        {
            set_mot_duty(saved.mot_spd, 0.0f);
        }
        break;

    case bt_A_Dn_0:
        if (saved.mot_spd > 0)
            saved.mot_spd--;
        if (mot_out > 0) // 走行中
        {
            set_mot_duty(saved.mot_spd, 0.0f);
        }
        break;

    case bt_A_Up:
        if (saved.mot_spd < MOTMAX)
            saved.mot_spd++;
        set_mot_duty(saved.mot_spd, 0.0f);
        break;

    case bt_A_Dn:
        if (saved.mot_spd > 0)
            saved.mot_spd--;
        set_mot_duty(saved.mot_spd, 0.0f);
        break;

    case bt_A_R:
        if (saved.str0 < STR_ADJ_MAX)
            saved.str0++;
        set_str_cmd(0.0f, 0.0f); // + = Right turn
        set_mot_duty(0.0f, 0.0f);
        break;

    case bt_A_L:
        if (saved.str0 > STR_ADJ_MIN)
            saved.str0--;
        set_str_cmd(0.0f, 0.0f); // - = Left turn
        set_mot_duty(0.0f, 0.0f);
        break;

    case bt_S_Up:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str < STR_GA_MAX)
            saved.gain_str = ((int)(saved.gain_str * 1000) + 1) / 1000.f;
        break;

    case bt_S_Dn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str > STR_GA_MIN)
            saved.gain_str = ((int)(saved.gain_str * 1000) - 1) / 1000.f;
        break;

    case bt_R_Up:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_w_roll < ROLL_GA_MAX)
            saved.gain_w_roll = ((int)(saved.gain_w_roll * 1) + 1);
        break;

    case bt_R_Dn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_w_roll > ROLL_GA_MIN)
            saved.gain_w_roll = ((int)(saved.gain_w_roll * 1) - 1);
        break;

    case bt_D_St:
        set_mot_duty(0.0f, 0.0f);
        break;

    case bt_D_Up:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str_diff < DIFF_GA_MAX)
            saved.gain_str_diff = ((int)(saved.gain_str_diff * 1000) + 1) / 1000.f;
        break;

    case bt_D_Dn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str_diff > DIFF_GA_MIN)
            saved.gain_str_diff = ((int)(saved.gain_str_diff * 1000) - 1) / 1000.f;
        break;

    case bt_Std_nutUp:
        set_mot_duty(0.0f, 0.0f);
        if (saved.ang_std_nut < EX1MAX)
            saved.ang_std_nut += 1;
        set_ex1_angle(saved.ang_std_nut, 0.1f);
        break;

    case bt_Std_nutDn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.ang_std_nut > EX1MIN)
            saved.ang_std_nut -= 1;
        set_ex1_angle(saved.ang_std_nut, 0.1f);
        break;

    case bt_str_turnUp:
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_turn < STRMAX)
            saved.str_turn += 1;
        break;

    case bt_str_turnDn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_turn > (STRMAX / 10))
            saved.str_turn -= 1;
        break;

    case bt_strcmd_Up:
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_cmd_speed < 100)
            saved.str_cmd_speed++;
        break;

    case bt_strcmd_Dn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_cmd_speed > 0)
            saved.str_cmd_speed--;
        break;

    case IR_ON:
        set_mot_duty(0.0f, 0.0f);
        autoCircling = true;
        break;

    case IR_OFF:
        set_mot_duty(0.0f, 0.0f);
        autoCircling = false;
        break;

    case bt_yaw_coeffUp:
        set_mot_duty(0.0f, 0.0f);
        if (saved.yaw_coeff < GYDIR_YAW_MAX)
            saved.yaw_coeff += 0.001f;
        if (saved.yaw_coeff > GYDIR_YAW_MAX)
            saved.yaw_coeff = GYDIR_YAW_MAX;
        break;

    case bt_yaw_coeffDn:
        set_mot_duty(0.0f, 0.0f);
        if (saved.yaw_coeff > GYDIR_YAW_MIN)
            saved.yaw_coeff -= 0.001f;
        if (saved.yaw_coeff < GYDIR_YAW_MIN)
            saved.yaw_coeff = GYDIR_YAW_MIN;
        break;

    case bt_Ld_Default:
        set_mot_duty(0.0f, 0.0f);
        uint32_t opt = saved.op_time_s;
        saved = savedefault;
        saved.op_time_s = opt;
        break;

    default: // Only the last command is executed, so do nothing here.
        xQueueOverwrite(control_queue, &msg->id);
        break;
    }
    return res;
}
