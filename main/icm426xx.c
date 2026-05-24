/*
    IMU datasheet
   *ICM-42607   https://www.lcsc.com/datasheet/C2879807.pdf
    ICM-40608   https://www.lcsc.com/datasheet/C2833762.pdf
    ICM-42670   https://www.lcsc.com/datasheet/C3288646.pdf
*/
#include "userdefine.h"
static const char TAG[] = "icm426xx";

#define IMU_ADDR 0x68      // AD0 = GND
#define PWR_MGMT0 0x1F     // Power management settings
#define ACCEL_DATA_X1 0x0B // Accelerometer X-axis high byte
#define GYRO_CONFIG0 0x20  // [6,5] GYRO_UI_FS_SEL [3,0] GYRO_ODR
#define ACCEL_CONFIG0 0x21 //
#define ACCEL_CONFIG1 0x22 //
#define GYRO_CONFIG1 0x23  // [2,0] GYRO_UI_FILT_BW

#define WHO_AM_I 0x75             // Device ID register
#define ACC_LOPASS_NON 0x00       // ODR=1.6kHz=800Hz
#define ACC_LOPASS_40HZ 0x07      //
#define I2C_MASTER_TIMEOUT 100    // msec

#define GY_SENSITIVITY (1.0f / 65.5f)       // deg/sec/LSB  ±500/dps
#define GRAVITY 9.80665f                    //
#define AC_SENSITIVITY (GRAVITY / 16384.0f) // ±2g設定時
#define I2C_CLOCK_SPD 1200000               // Hz for 1.0MHz

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static esp_err_t i2c_write(uint8_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev_handle, buf, len + 1, I2C_MASTER_TIMEOUT);
}

static esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, I2C_MASTER_TIMEOUT);
    i2c_master_bus_reset(bus_handle);

    return ret;
}

// Accel+-  0:16g 1:8g 2:4g 3:2g
// Gyro+-   0:250dps 1:500dps 2:1000dps 3:2000dps
static void icm426xx_get_fs()
{
    uint8_t data;

    i2c_read(ACCEL_CONFIG0, &data, 1);
    data = (data >> 5) & 0x07;
    ESP_LOGI(TAG, "Accel FS=%d", data);
    i2c_read(GYRO_CONFIG0, &data, 1);
    data = (data >> 5) & 0x07;
    ESP_LOGI(TAG, "Gyro FS=%d", data);
}

static void read_who_am_i()
{
    uint8_t who_am_i = 0;
    if (i2c_read(WHO_AM_I, &who_am_i, 1) == ESP_OK)
    {
        ESP_LOGI(TAG, "WHO_AM_I: 0x%02x", who_am_i);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I reg");
    }
}

void icm426xx_sleep()
{
    uint8_t sleep_mode = 0x00; // GYRO_MODE=00 (off), ACCEL_MODE=00 (off), IDLE=0, ACCEL_LP_CLK_SEL=0
    ESP_LOGI(TAG, "entering sleep (PWR_MGMT0=0x%02X)", sleep_mode);
    i2c_write(PWR_MGMT0, &sleep_mode, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////
static uint8_t raw[12] = {0};
static uint8_t addr[1] = {0};
static volatile bool i2c_done = true;

// call this to start reading
void icm426xx_start_read()
{
    addr[0] = ACCEL_DATA_X1;
    if (i2c_done)
    {
        i2c_done = false;
        i2c_master_transmit_receive(dev_handle, addr, 1, raw, 12, 0);
    }
}

// I2C end of read data
static bool IRAM_ATTR i2c_trans_done_callback(i2c_master_dev_handle_t dev_handle,
                                              const i2c_master_event_data_t *event_data,
                                              void *user_ctx)
{
    i2c_done = true;
    return false;
}

// Call this after icm426xx_start_read()
void icm426xx_get_data(Tvector6d *pac)
{
    uint32_t timeout = 0;

    while (!i2c_done && timeout < 2000)
    { // timeout = 20ms
        esp_rom_delay_us(10);
        timeout++;
    }
    pac->x = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]) * AC_SENSITIVITY;
    pac->y = (int16_t)(((uint16_t)raw[2] << 8) | (uint16_t)raw[3]) * AC_SENSITIVITY;
    pac->z = (int16_t)(((uint16_t)raw[4] << 8) | (uint16_t)raw[5]) * AC_SENSITIVITY;
    pac->gx = (int16_t)(((uint16_t)raw[6] << 8) | (uint16_t)raw[7]) * GY_SENSITIVITY;
    pac->gy = (int16_t)(((uint16_t)raw[8] << 8) | (uint16_t)raw[9]) * GY_SENSITIVITY;
    pac->gz = (int16_t)(((uint16_t)raw[10] << 8) | (uint16_t)raw[11]) * GY_SENSITIVITY;
}

void icm426xx_init()
{
    uint8_t cfg;

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = I2C_CLOCK_SPD,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    cfg = 0x45; // GYRO_UI_FS_SEL=500dps(0100), GYRO_ODR=1.6KHz(0101)
    i2c_write(GYRO_CONFIG0, &cfg, 1);

    cfg = 0x65; // 0b01100101
    i2c_write(ACCEL_CONFIG0, &cfg, 1);

    cfg = ACC_LOPASS_NON; // bypass Lo-pass
    i2c_write(GYRO_CONFIG1, &cfg, 1);

    cfg = ACC_LOPASS_40HZ; // 40Hz Lo-pass
    i2c_write(ACCEL_CONFIG1, &cfg, 1);

    // --- Power ON (Accel + Gyro ON) ---
    cfg = 0x0F;
    i2c_write(PWR_MGMT0, &cfg, 1);

    read_who_am_i();
    icm426xx_get_fs();

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 2026.03.29 ADD

    i2c_master_bus_config_t bus_configas = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_configas, &bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    // コールバックの登録
    i2c_master_event_callbacks_t cbs = {
        .on_trans_done = i2c_trans_done_callback,
    };
    ESP_ERROR_CHECK(i2c_master_register_event_callbacks(dev_handle, &cbs, NULL));
}
