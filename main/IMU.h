typedef struct
{
    float x;
    float y;
    float z;
    float gx;
    float gy;
    float gz;
} Tvector6d;

extern Tvector6d acc;        // Last IMU data Direction of gravity: Vert Dn:x+, Left Dn:z+, Nose Dn:y+
extern Tvector6d acc_offset; // Offsets of IMU data

extern volatile bool gy_auto_cal_done;
extern float *pyaw_coeff;
extern uint32_t PWM_delay_us; // PLL for synchronizing PWM timer to IMU ODR cycle

esp_err_t IMU_init();
extern void (*IMU_sleep)(); // change functions for PCB version
extern void (*IMU_startRead)();
extern void (*IMU_getData)(Tvector6d *pac);
extern uint32_t sv_freq; // [Hz] Servo control calculation period = Servo signal frame period

////////////////////////////////////////////////
#define SPI_SWAP_16(val) __builtin_bswap16(val)
#define IMU_ODR_TERM 625   // us 1/ODR=625us, 1600Hz=625us

////////////////////////////////////////////////////////////////
// Define macros corresponding to physical orientation based on IMU output axis definition.
// If the mounting direction of the IMU is changed, modify only this section.
// Direction vector indicating the IMU ROLL axis
////////////////////////////////////////////////////////////////
// IMU output axis definition: Unified to Right-Hand Rule
////////////////////////////////////////////////////////////////

///////////// Angular velocity (deg/s)
// X-axis (forward): Right tilt = Positive (Inverted because IMU is mounted facing backward)
#define GY_ROLL (-(acc.gy - acc_offset.GYOFFSET_ROLL))     // Latest value
#define GY_ROLL_P(p) (-(p->gy - acc_offset.GYOFFSET_ROLL)) // When retrieving from array/pointer
// Y-axis (rightward): Nose up = Positive (Direct mapping)
#define GY_PITCH (acc.gz - acc_offset.GYOFFSET_PITCH)
#define GY_PITCH_P(p) (p->gz - acc_offset.GYOFFSET_PITCH)
// Z-axis (downward): Right turn = Positive (Inverted because IMU is facing upward)
#define GY_YAW (-(acc.gx - acc_offset.GYOFFSET_YAW))
#define GY_YAW_P(p) (-(p->gx - acc_offset.GYOFFSET_YAW))

//////////// Acceleration (m/s2)
#define ACC_X(p) (-p->y) // head
#define ACC_Y(p) (p->z)  // right
#define ACC_Z(p) (-p->x) // down

// --- Calculation element mappings below ---
#define GY_YAW_ELM gx
#define GY_ROLL_ELM gy
#define GY_PITCH_ELM gz
#define LATERAL_G (acc.z)
////////////////////////////////////////////////////////////////

// IMU offset error (calibration values)
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