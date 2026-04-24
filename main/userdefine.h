/*////////////////////////////////////////////////////////////////////////////////
userdefine.h        ROBOBIKE project

Copyright 2026.04.24 M.Tanaami
////////////////////////////////////////////////////////////////////////////////*/

#pragma once

#define PROGVER 1029 // version for program  IDF ver. 5.4.1
#define DATAVER 1022 // version for saved data in NVS

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
#define IO_LED1 3 // LED Hi:ON Lo:OFF
#define GPIO_DRV 4
#define GPIO_STR 5
#define GPIO_STR_IN 8
#define GPIO_EX1 9
#define IO_10 10
#define IO_20 20
#define IO_21 21
#define IO_SV_EN 10

///////////////// R/C servo using /////////////////
#define SERVO_NEUTRAL_DUTY (1500.0f)       // 0deg 1500us
#define STR_ADJ_MIN (-20)                  //
#define STR_ADJ_MAX (20)                   //
#define STR_CMD_SPD_P (45.0f / SERVO_FREQ) // deg/s positive
#define STR_CMD_SPD_N (25.0f / SERVO_FREQ) // deg/s negative
#define STR_SLIDER_MAX 100.0f              // +-100
#define STR_GA_MAX 0.200f                  //
#define STR_GA_MIN 0.001f                  //
#define DIFF_GA_MAX 0.200f                 //
#define DIFF_GA_MIN 0.000f                 //
#define ROLL_GA_MAX 60                     //
#define ROLL_GA_MIN 1                      //
#define ANG2PULSE (1000.0f / 90.0f)        //

typedef enum // 自動旋回修正用状態定義
{
    rsOuter,         // 外側走行、定常旋回
    rsInner_Correct, // 内側走行、修正動作
    rsInner_Stable   // 内側走行、定常旋回
} TRunState;

#define AUTOCORRECTTIME 150 // msec
#define AUTOCORRECTRATE 0.6f

typedef struct
{
    int ver;              // data ver
    uint32_t op_time_s;   //
    bool isChecked;       //
    float gain_str;       // delta = 0.01
    float gain_str_diff;  //
    float gain_w_roll;    // delta = 1.0
    Tvector6d acc_offset; //
    Tvector6d acc_dir;    //
    float str_diff_alph;  //
    int str0;             //
    int mot_spd;          //
    int ang_std_nut;      //
    float run_coeff;      // run-speed feedback coefficient
    float yaw_coeff;      // yaw‑rate feedback coefficient
    int str_turn;         //
    uint32_t CRC;         //
} TSave;

#define STRMAX 65
#define STR_STOP 25
#define MOTMAX 60 // MG90D max duty = 90%,21.1kHz @50deg()
#define MOT_SPEED_BACK (-20)
#define EX1MAX 60
#define EX1MIN (-10)
#define STD_RUN 80
#define STD_STD_NUT 14 // diff STD. - NUT.

#define LEDHIGH 255
#define LEDLOW 16
#define SLEEP_DURATION_MS (10 * 60 * 1000UL) // n minuits to sleep

extern volatile uint32_t millis;
extern float mot_cmd;  // -100 <> +100%
extern float mot_step; // %/step
extern float mot_out;  // extern float str_cmd0; // -180 / +180 deg
extern float str_cmd0; // -180 / +180 deg
extern float str_cmd1; // -180 / +180 deg
extern float str_cmd2; // -180 / +180 deg
extern float str_out;  // -180 / +180 deg
extern float ex1_cmd;  // -180 / +180 deg
extern float ex1_out;  // -180 / +180 deg

extern TSave saved;
extern const TSave savedefault;

extern bool auto_en;
extern bool autoCircling; // do auto circling

extern volatile uint32_t userLastControlTime;

// webserver handlers
extern const httpd_uri_t root;
extern const httpd_uri_t command;
extern const httpd_uri_t setup;
extern const httpd_uri_t setup2;
extern const httpd_uri_t monitor;
extern const httpd_uri_t get_acc;
extern const httpd_uri_t favicon;
extern const httpd_uri_t generate_204;
extern const httpd_uri_t hotspot;
extern const httpd_uri_t ncsi;
extern const httpd_uri_t generate_204;
extern const httpd_uri_t ota_update;
extern const httpd_uri_t ota;

///////////////// common function declarations ////////////////
bool isNms(uint32_t *lastNms, uint32_t Nms);

void set_mot_duty(float duty, float step);   // duty in +-100%
void set_str_cmd(float angle, float step);   // step: deg/cycle
void set_ex1_angle(float angle, float step); // angle in deg, step in usec
void wait_mot_duty();
void wait_str_angle();
void wait_ex1_angle();

void deepSleep(uint32_t msec);
void auto_enable();
void auto_disable();

void set_led_brightness(uint8_t);
void waitms(uint32_t);

void savenvs();

void webserver_start();

char *SysID();
char *mkcsv();
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);
void put_data(Tvector6d *new_data);
const char *get_data();
