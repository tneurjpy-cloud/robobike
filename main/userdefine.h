/*////////////////////////////////////////////////////////////////////////////////
userdefine.h        ROBOBIKE project

Copyright 2026.06.05 M.Tanaami
////////////////////////////////////////////////////////////////////////////////*/

#pragma once

#define PROGVER 1032 // version for program
<<<<<<< HEAD
#define DATAVER 4    // version for saved data in NVS
=======
#define DATAVER 4 // version for saved data in NVS
>>>>>>> ebc98efa1e17eb97453bb0284e1236425478b1d6

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/gptimer.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

#include <esp_attr.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <dns_server.h>

#include "IMU.h"

#define ESP_WIFI_PASS ""

///////////////// GPIO use ////////////////
#define IO_0 0
#define IO_1 1
#define IO_2 2
#define IO_3 3 // LED Hi:ON Lo:OFF
#define IO_4 4
#define IO_5 5
#define IO_6 6
#define IO_7 7
#define IO_8 8
#define IO_9 9
#define IO_10 10
#define IO_20 20
#define IO_21 21

#define I2C_MASTER_SCL_IO IO_7
#define I2C_MASTER_SDA_IO IO_6
#define GPIO_DRV IO_4
#define GPIO_STR IO_5
#define GPIO_STR_IN IO_8
#define GPIO_EX1 IO_9
#define IO_SV_EN IO_10

///////////////// R/C servo using /////////////////
#define SERVO_NEUTRAL_DUTY (1500.0f) // 0deg 1500us
#define STR_ADJ_MIN (-20)            //
#define STR_ADJ_MAX (20)             //
#define STR_SLIDER_MAX 100.0f        // +-100
#define STR_GA_MAX 0.200f            //
#define STR_GA_MIN 0.001f            //
#define DIFF_GA_MAX 0.200f           //
#define DIFF_GA_MIN 0.000f           //
#define ROLL_GA_MAX 60               //
#define ROLL_GA_MIN 1                //
#define ANG2PULSE (1000.0f / 90.0f)  //

#define AUTOCORRECTTIME 150 // msec
#define AUTOCORRECTRATE 0.6f

typedef struct
{
    int ver;              // data ver
    uint32_t op_time_s;   // operation time in minutes
    bool isChecked;       //
    float gain_str;       // delta = 0.01
    float gain_str_diff;  //
    float gain_w_roll;    // delta = 1.0
    Tvector6d acc_offset; //
    Tvector6d acc_dir;    //
    float str_diff_alph;  // alpha for low-pass filter of str_dev_diff
    int str0;             //
    int mot_spd;          //
    int ang_std_nut;      //
    float run_coeff;      // run-speed feedback coefficient
    float yaw_coeff;      // yaw‑rate feedback coefficient
    int str_turn;         //
    int str_cmd_speed;    // steering command changeing speed deg/sec
    uint32_t CRC;         //
} TSave;

typedef enum // 自動旋回修正用状態定義
{
    rsOuter,         // 外側走行、定常旋回
    rsInner_Correct, // 内側走行、修正動作
    rsInner_Stable   // 内側走行、定常旋回
} TRunState;

#define STRMAX 65
#define STR_STOP 35
#define MOTMAX 60 // MG90D max duty = 90%,21.1kHz @50deg()
#define MOT_SPEED_BACK (-20)
#define EX1MAX 60
#define EX1MIN (-10)
#define STD_RUN 80
#define STD_STD_NUT 14 // diff STD. - NUT.

#define RING_BUF_SIZE (SAMPLE_RATE_HZ * 5) // for data monitor
#define CTL_DATA_BUFSIZE (65536)           //

#define LEDHIGH 255
#define LEDLOW 8
#define SLEEP_DURATION_MS (30 * 60 * 1000UL) // 30 minutes to sleep
#define millis() ((uint32_t)(esp_timer_get_time() / 1000))
#define waitTaskms(xms) vTaskDelay(pdMS_TO_TICKS(xms))

extern float mot_cmd;    // -100 <> +100%
extern float mot_step;   // %/step
extern float mot_out;    // extern float str_cmd0; // -180 / +180 deg
extern float str_cmd0;   // -180.0f / +180.0f deg  > 0.0: turn right
extern float str_cmd1;   // -180 / +180 deg
extern float str_target; // -180 / +180 deg
extern float str_out;    // -180 / +180 deg
extern float ex1_cmd;    // -180 / +180 deg
extern float ex1_out;    // -180 / +180 deg

extern TSave saved;
extern const TSave savedefault;

extern bool auto_en;
extern bool autoCircling; // do auto circling

extern volatile uint32_t userLastControlTime;

// webserver handlers
extern const httpd_uri_t root;
extern const httpd_uri_t command;
extern const httpd_uri_t get_acc;
extern const httpd_uri_t generate_204;
extern const httpd_uri_t hotspot;
extern const httpd_uri_t ncsi;
extern const httpd_uri_t generate_204;
extern const httpd_uri_t ota_update;
extern const httpd_uri_t ota;

// control command
#define COMMAND_LIST(V) \
    V(stp_all)          \
    V(only_data)        \
    V(bt_F)             \
    V(bt_S)             \
    V(bt_L)             \
    V(bt_R)             \
    V(bt_Str_S)         \
    V(bt_BK)            \
    V(bt_A_Up_0)        \
    V(bt_A_Dn_0)        \
    V(bt_A_Up)          \
    V(bt_A_Dn)          \
    V(bt_A_L)           \
    V(bt_A_R)           \
    V(bt_S_Up)          \
    V(bt_S_Dn)          \
    V(bt_R_Up)          \
    V(bt_R_Dn)          \
    V(bt_D_St)          \
    V(bt_D_Up)          \
    V(bt_D_Dn)          \
    V(bt_Std_nutAuto)   \
    V(bt_Std_nutUp)     \
    V(bt_Std_nutDn)     \
    V(bt_str_turnUp)    \
    V(bt_str_turnDn)    \
    V(bt_strcmd_Up)     \
    V(bt_strcmd_Dn)     \
    V(IR_ON)            \
    V(IR_OFF)           \
    V(bt_yaw_coeffUp)   \
    V(bt_yaw_coeffDn)   \
    V(bt_Ld_Default)

typedef enum
{
    unknown = 0,
#define AS_ENUM(name) name,
    COMMAND_LIST(AS_ENUM)
#undef AS_ENUM
} TcmdID;

typedef struct
{
    TcmdID id;
    httpd_req_t *req;
} control_msg_t;

///////////////// common function declarations ////////////////
void set_mot_duty(float duty, float step); // duty in +-100%, step: /cycle
void wait_mot_duty();
void set_str_cmd(float angle, float step); // angle in deg, step: /cycle
void wait_str_angle();
void set_ex1_angle(float angle, float step); // angle in deg, step: /cycle
void wait_ex1_angle();
void savenvs();
void showTasks(void);
const char *cmdID_to_str(TcmdID id);
void webserver_start(void);
char *SysID(void);
void set_led_brightness(uint8_t brightness);
uint8_t get_led_brightness();
void deepSleep(uint32_t ms);
void auto_disable(void);
void auto_enable(void);
void waitms(uint32_t t);
bool isNms(uint32_t *lastNms, uint32_t Nms);
void userdeviceinit(void);
void servo_init(void);
void control_init(void);
void do_str_cmd_calc(void);
void put_control_data(void);
esp_err_t put_command(control_msg_t *msg);
char *get_edit_data(void);
char *get_control_data(void);
void str_pwm_out(float angle);
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);
