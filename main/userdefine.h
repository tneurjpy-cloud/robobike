/*////////////////////////////////////////////////////////////////////////////////
userdefine.h        ROBOBIKE project

* How to set up IntelliSense in VS Code:
(1) Install the ESP-IDF Extension.
(2) (Ctrl+Shift+P)Run the command "ESP-IDF: Add vscode configuration folder".
(3) Perform a Build to generate necessary configuration files.

* Settings from menuconfig are saved to \project\sdkconfig
* Change via menuconfig, KCONFIG Name:
    configTICK_RATE_HZ 1000(Hz)
    Flash size : 4MB
    Partition Table : Custom Partition Table CSV
        CSV file : partitions.csv
    configUSE_TRACE_FACILITY : chaecked
    configGENERATE_RUN_TIME_STATS : checked

* MOTOR OUTPUT = 70-90rpm
* ESP-IDF components and tools paths depend on installation location
*
******** TODO ********
- Operation time
- Rotary encoder implementation

* Change Log
Date        CODE    DATA    Description
2026.       2029    2029    Add: ota, data monitor. Chg: TVector6d
2026.04.07  2028    2022    Add: WiFi AP Ch. randomize
2026.03.31  1027    2021    Add: BACK cmd, SV EN Cont. heapless buffer
2026.03.20  1025    2021    Move: GPIO definitions from servo.c to userdefine.h
2026.03.01  1024    2021    Add: IMU data API, dbg. calibration
2026.02.09  1023    2021    Add: Auto circling On/Off, restore SLEEP function
2026.01.20  1022    2021    Add: Speed buttons to control UI, add only_data
2026.01.17  1021    1021    Add: Auto circling
2026.01.11  1020    1020    The 1st release
2025.12.10  1017    1015    Ex1 step: converted to float (smooth side stand movement)
2025.11.29  1016    1015    Add: Steering slide bar
2025.11.11  1014    1014    Suppress UI scaling, add adjustment items, update stop sequence
2025.10.28  1011    1011    Update: Adjustment screen
////////////////////////////////////////////////////////////////////////////////*/

#pragma once

#define PROGVER 1029 // version for program  IDF ver. 5.4.1
#define DATAVER 1022 // version for saved data in NVS
#define ESP32BINMARK 0xE9

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

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <dns_server.h>
#include <esp_ota_ops.h>

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
#define STR_CMD_SPD_P (40.0f / SERVO_FREQ) // deg/s positive
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
#define STR_STOP 30
#define MOTMAX 60 // MG90D max duty = 90%,21.1kHz @50deg()
#define MOT_SPEED_BACK (-20)
#define EX1MAX 60
#define EX1MIN (-10)
#define STD_RUN 80
#define STD_STD_NUT 12 // diff STD. - NUT.

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
