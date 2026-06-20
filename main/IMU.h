typedef struct
{
    float x;
    float y;
    float z;
    float gx;
    float gy;
    float gz;
} Tvector6d;

extern Tvector6d acc;        // last IMU data Direction of gravity:: Vert Dn:x+, Left Dn:z+, Nose Dn:y+
extern Tvector6d acc_offset; // offsets of IMU data

extern volatile bool gy_auto_cal_done;
extern float *pyaw_coeff;

//////////////// icm426xx using ////////////////
void icm426xx_init();
void icm426xx_sleep();
void icm426xx_start_read();
void icm426xx_get_data(Tvector6d *pac);
#define IMU_init() icm426xx_init()
#define IMU_sleep() icm426xx_sleep()
#define IMU_startRead() icm426xx_start_read()
#define IMU_getData(p) icm426xx_get_data(p)
////////////////////////////////////////////////

#define SV_FRQ 250 // [Hz] サーボ制御計算周期 = サーボ信号フレーム周期
#define SAMPLE_RATE_HZ SV_FRQ
#define SAMPLE_COUNT (3 * SAMPLE_RATE_HZ)

////////////////////////////////////////////////////////////////
// IMUの出力軸定義に基づいて、物理的意味に対応するマクロを定義する
// IMUの取り付け方向を変えたら、ここだけを変更すればよい
// IMUのROLL軸を示す方向ベクトル
////////////////////////////////////////////////////////////////
// IMUの出力軸定義：右手座標系（右ネジの法則）へ統一
////////////////////////////////////////////////////////////////

///////////// 角速度 (deg/s)
// X軸（前向き）: 「右傾斜＝＋」 IMUが後向きのため反転
#define GY_ROLL (-(acc.gy - acc_offset.GYOFFSET_ROLL))
#define GY_ROLL_P(p) (-(p->gy - acc_offset.GYOFFSET_ROLL))
// Y軸（右向き）: 「頭上げ＝＋」 そのまま
#define GY_PITCH (acc.gz - acc_offset.GYOFFSET_PITCH)
#define GY_PITCH_P(p) (p->gz - acc_offset.GYOFFSET_PITCH)
// Z軸（下向き）: 「右旋回＝＋」 IMUが上向きのため反転
#define GY_YAW (-(acc.gx - acc_offset.GYOFFSET_YAW))     // 最新の値
#define GY_YAW_P(p) (-(p->gx - acc_offset.GYOFFSET_YAW)) // 配列等から取出すときはこれで

//////////// 加速度 (m/s2)
#define ACC_X(p) (-p->y) // head
#define ACC_Y(p) (p->z)  // right
#define ACC_Z(p) (-p->x) // down

// --- 以下、計算要素の紐付け ---
#define GY_YAW_ELM gx
#define GY_ROLL_ELM gy
#define GY_PITCH_ELM gz
#define LATERAL_G (acc.z)
////////////////////////////////////////////////////////////////

// IMUのオフセット誤差（較正値）
#define GYOFFSET_YAW gx
#define GYOFFSET_ROLL gy
#define GYOFFSET_PITCH gz
#define DELTA_ROLL dy
////////////////////////////////////////////////////////////////

#define GYDIR_YAW (*pyaw_coeff)
#define GYDIR_ROLL 0.9996f

#define GYDIR_YAW_MIN (-0.02f)
#define GYDIR_YAW_MAX (0.02f)

float IMU_roll();
float IMU_side_acc();
bool IMU_getZero();
