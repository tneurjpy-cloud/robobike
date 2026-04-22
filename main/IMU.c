////////////////////////////////////////////////////////////////////////
/*
    IMU.c   for MPU6050 / ICM-42607 6-axis acc./gyro. sensor
*/
////////////////////////////////////////////////////////////////////////
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <string.h>
#include "IMU.h"

//////////////////////////////////////////////////////////////
// device specific functions
extern void icm426xx_init();
extern void icm426xx_sleep();
extern void icm426xx_get_data(Tvector6d *pac);

#define IMU_getData() icm426xx_get_data(&acc)
#define IMU_initIMU() icm426xx_init()
#define IMU_SLEEPIMU() icm426xx_sleep()
//////////////////////////////////////////////////////////////

Tvector6d acc = {0, 0, 0, 0, 0, 0};        // accel raw data
Tvector6d acc_offset = {0, 0, 0, 0, 0, 0}; //

// for automatic calibration
volatile bool gy_auto_cal_done = false;

// Determin the roll axis vector
// 車体を実際に何度もロールさせて、ロール軸を実測する
#ifdef COMPUTE_ROLL_AXIS
static Tvector6d gyro_data[SAMPLE_COUNT]; // array of gyro data
void compute_roll_axis_direction(Tvector6d *data, int count)
{
    Tvector6d sum = {0., 0., 0.};

    pacc->x = 0.;
    pacc->y = 0.;
    pacc->z = 0.;
    for (int i = 0; i < count; i++) // average == offset
    {
        pacc->x += data[i].x;
        pacc->y += data[i].y;
        pacc->z += data[i].z;
    }
    pacc->x /= count;
    pacc->y /= count;
    pacc->z /= count;

    for (int i = 0; i < count; i++) // 方向ベクトルを抽出
    {
        float dx = data[i].x - pacc->x;
        float dy = data[i].y - pacc->y;
        float dz = data[i].z - pacc->z;
        if (DELTA_ROLL >= 0.0f) // y軸（ロール軸）方向の符号で方向を揃える
        {
            sum.x += dx;
            sum.y += dy;
            sum.z += dz;
        }
        else
        {
            sum.x -= dx;
            sum.y -= dy;
            sum.z -= dz;
        }
    }
}

// Calibrate independently 5sec to do this
void calibrate_roll()
{
    { // get memory using flag
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    getZeroDoing = true;
    for (int i = 0; i < SAMPLE_COUNT; i++) // sample data
    {
        IMU_getGyro();
        gyro_data[i] = gy;
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }
    getZeroDoing = false;
}
#endif

// 自動制御offの時、5msごとに呼び出してゼロ点較正
void IMU_getZero()
{
    static int gyavecount = 0;
    static Tvector6d gysum;

    float temp = IMU_roll();
    if (fabsf(temp) < 2.0f)
    {
        gysum.gx += acc.gx; // acc is raw data
        gysum.gy += acc.gy;
        gysum.gz += acc.gz;
        gyavecount++;
        if (gyavecount >= SAMPLE_COUNT) // get 5sec average
        {                               // calc average
            acc_offset.gx = gysum.gx / SAMPLE_COUNT;
            acc_offset.gy = gysum.gy / SAMPLE_COUNT;
            acc_offset.gz = gysum.gz / SAMPLE_COUNT;
            gyavecount = 0;
            gy_auto_cal_done = true;
            gysum = (Tvector6d){0, 0, 0, 0, 0, 0};
        }
    }
    else
    { // detect noise
        gyavecount = 0;
        gysum = (Tvector6d){0, 0, 0, 0, 0, 0};
    }
    return;
}

/////////////////////////////////////////////////////////////////////
// Use device specific functions
/////////////////////////////////////////////////////////////////////
void IMU_init()
{
    IMU_initIMU();
}

// deg/sec 3-axis calibrated
float IMU_roll()
{
    IMU_getData();
    // Dot product of the roll axis and angular velocity vector
    return (GY_YAW - acc_offset.GYOFFSET_YAW) * GYDIR_YAW + // for stabilizing effect
           (GY_ROLL - acc_offset.GYOFFSET_ROLL) * GYDIR_ROLL;
}

void IMU_sleep()
{
    IMU_SLEEPIMU();
}