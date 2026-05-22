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

#define LOG_FORMAT_HEADER "time,steer,speed,accel_x"

#define SV_FRQ 250 // [Hz] サーボ制御計算周期 = サーボ信号フレーム周期
#define SAMPLE_RATE_HZ SV_FRQ
#define SAMPLE_COUNT (3 * SAMPLE_RATE_HZ)
#define BUFFER_SECONDS 1

////////////////////////////////////////////////////////////////
// IMUの出力軸定義に基づいて、物理的意味に対応するマクロを定義する
// IMUの取り付け方向を変えたら、ここだけを変更すればよい
// IMUのROLL軸を示す方向ベクトル
////////////////////////////////////////////////////////////////
// IMUの出力軸定義：右手座標系（右ネジの法則）へ統一
////////////////////////////////////////////////////////////////

// X軸（上向き）: 右旋回でマイナスが出るため反転「右旋回＝正」
#define GY_YAW_RAW (acc.gx)                                // IMUの生データ
#define GY_YAW (-(GY_YAW_RAW - acc_offset.GYOFFSET_YAW))   // 最新の値
#define GY_YAW_P(pa) (-(pa->gx - acc_offset.GYOFFSET_YAW)) // 配列等から取出すときはこれで

// Y軸（後向き）: 左傾斜でプラスが出るため反転「右傾斜＝正」
#define GY_ROLL_RAW (acc.gy)
#define GY_ROLL (-(GY_ROLL_RAW - acc_offset.GYOFFSET_ROLL))
#define GY_ROLL_P(pa) (-(pa->gy - acc_offset.GYOFFSET_ROLL))

// Z軸（右向き）: 頭上げ＝正」そのまま
#define GY_PITCH_RAW (acc.gz)
#define GY_PITCH (GY_PITCH_RAW - acc_offset.GYOFFSET_PITCH)
#define GY_PITCH_P(pa) (pa->gz - acc_offset.GYOFFSET_PITCH)

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
#define GYDIR_PITCH 0.0f

#define GYDIR_YAW_MIN (-0.02f)
#define GYDIR_YAW_MAX (0.02f)

void IMU_init();
float IMU_roll();
void IMU_startRead();
float IMU_side_acc();
void IMU_sleep();
bool IMU_getZero();
