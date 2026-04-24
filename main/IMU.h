typedef struct
{
    float x;
    float y;
    float z;
    float gx;
    float gy;
    float gz;
} Tvector6d;

extern Tvector6d acc;        // Direction of gravity:: Vert Dn:x+, Left Dn:z+, Nose Dn:y+
extern Tvector6d acc_offset; // offsets of IMU data

extern volatile bool gy_auto_cal_done;
extern float *pyaw_coeff;

#define LOG_FORMAT_HEADER "time,steer,speed,accel_x"

#define SERVO_FREQ 200 // [Hz] サーボ制御計算周期 = サーボ信号フレーム周期
#define SAMPLE_RATE_HZ SERVO_FREQ
#define SAMPLE_COUNT (5 * SAMPLE_RATE_HZ)
#define BUFFER_SECONDS 1

// IMUの出力軸定義に基づいて、物理的意味に対応するマクロを定義する
// IMUの取り付け方向を変えたら、ここだけを変更すればよい
// IMUのROLL軸を示す方向ベクトル
#define GY_YAW (acc.gx)    // left turn is positive
#define GY_ROLL (acc.gy)   // left tilt is positive
#define GY_PITCH (acc.gz)  // pitch up is positive
#define PGY_YAW (pa->gx)   // left turn is positive
#define PGY_ROLL (pa->gy)  // left tilt is positive
#define PGY_PITCH (pa->gz) // pitch up is positive

#define GYDIR_YAW (*pyaw_coeff)
#define GYDIR_ROLL 0.9996f
#define GYDIR_PITCH 0.0f

#define LATERAL_G (acc.z)

#define GYDIR_YAW_MIN (0.0f)
#define GYDIR_YAW_MAX (0.02f)

// IMUのオフセット誤差（較正値）
#define GYOFFSET_YAW gx
#define GYOFFSET_ROLL gy
#define GYOFFSET_PITCH gz
#define DELTA_ROLL dy

void IMU_init();
float IMU_roll();
float IMU_side_acc();
void IMU_sleep();
void IMU_getZero();
