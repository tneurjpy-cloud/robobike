#include "userdefine.h"

static const char TAG_SPI[] = "icm426xx_spi";

uint16_t imu_timestamp_us = 0;

#define ICM42670L_WHOAMI 0x63
#define PIN_CS IO_9
#define PIN_SCLK IO_8
#define PIN_SDA IO_7

#define REG_DEVICE_CONFIG 0x01
#define REG_ACCEL_DATA_X1 0x0B
#define REG_PWR_MGMT0     0x1F     
#define REG_GYRO_CONFIG0  0x20  
#define REG_ACCEL_CONFIG0 0x21 
#define REG_ACCEL_CONFIG1 0x22 
#define REG_GYRO_CONFIG1  0x23  
#define REG_INTF_CONFIG0  0x35  
#define REG_WHO_AM_I      0x75      

#define IMU_READ_BIT 0x80

#define GY_SENSITIVITY (1.0f / 65.5f)       // deg/sec/LSB  ±500 dps
#define GRAVITY 9.80665f                    //
#define AC_SENSITIVITY (GRAVITY / 16384.0f) // ±2g設定時 (m/s^2 単位)

static spi_device_handle_t spi_handle = NULL;

// 書き込み関数（command_bits = 8 を使用）
static esp_err_t icm426xx_write_reg_SPI(uint8_t reg, uint8_t val)
{
    if (spi_handle == NULL) return ESP_FAIL;

    uint8_t data_val = val;
    spi_transaction_t t = {
        .flags = 0,
        .cmd = reg & 0x7F, // Write時はMSB = 0
        .length = 8,       // データ部1バイト
        .tx_buffer = &data_val,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(spi_handle, &t);
}

static void read_attr()
{
    if (spi_handle == NULL)
        return;

    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = REG_WHO_AM_I | IMU_READ_BIT,
        .length = 0,
        .rxlength = 8,
    };

    if (spi_device_transmit(spi_handle, &t) == ESP_OK) {
        ESP_LOGI(TAG_SPI, "WHO_AM_I: 0x%02x", t.rx_data[0]);
    }
}

void icm426xx_sleep_SPI()
{
    if (spi_handle == NULL)
        return;

    ESP_LOGI(TAG_SPI, "entering sleep (PWR_MGMT0=0x00)");
    icm426xx_write_reg_SPI(REG_PWR_MGMT0, 0x00);
}

void icm426xx_start_read_SPI() {}

// 12バイトを1バイトずつ個別に安全に取得する方式
void icm426xx_get_data_SPI()
{
    if (spi_handle == NULL)
        return;

    uint8_t raw[12] = {0};

    // ACCEL_DATA_X1 (0x0B) から GYRO_DATA_Z0 (0x16) まで、1バイトずつ個別に読み出し
    for (int i = 0; i < 12; i++) {
        spi_transaction_t t = {
            .flags = SPI_TRANS_USE_RXDATA,
            .cmd = (REG_ACCEL_DATA_X1 + i) | IMU_READ_BIT,
            .length = 0,
            .rxlength = 8,
        };
        if (spi_device_transmit(spi_handle, &t) == ESP_OK) {
            raw[i] = t.rx_data[0];
        } else {
            return;
        }
    }

    // I2C版と同じデータ配置・スケーリングで処理
    int16_t *raw_16 = (int16_t *)raw;
    acc.x  = raw_16[0] * AC_SENSITIVITY;
    acc.y  = raw_16[1] * AC_SENSITIVITY;
    acc.z  = raw_16[2] * AC_SENSITIVITY;
    acc.gx = raw_16[3] * GY_SENSITIVITY;
    acc.gy = raw_16[4] * GY_SENSITIVITY;
    acc.gz = raw_16[5] * GY_SENSITIVITY;
}

esp_err_t icm426xx_init_SPI(void)
{
    IMU_sleep = icm426xx_sleep_SPI;
    IMU_startRead = icm426xx_start_read_SPI;
    IMU_getData = icm426xx_get_data_SPI;

    static spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_SDA,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32};

    // command_bits = 8 を有効にし、.cmdフェーズを利用する設定
    static spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000, 
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
        .command_bits = 8, // コマンドフェーズ（8ビット）を使用
        .address_bits = 0, 
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle));

    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 1: Set 3-wire mode in DEVICE_CONFIG
    icm426xx_write_reg_SPI(REG_DEVICE_CONFIG, 0x00);

    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2: Set initial configuration values
    icm426xx_write_reg_SPI(REG_GYRO_CONFIG0,  0x45);  // 500dps, 1.6kHz
    icm426xx_write_reg_SPI(REG_ACCEL_CONFIG0, 0x65);  // ±2g, 1.6kHz
    icm426xx_write_reg_SPI(REG_GYRO_CONFIG1,  0x00);  // bypass Lo-pass
    icm426xx_write_reg_SPI(REG_ACCEL_CONFIG1, 0x07);  // 40Hz Lo-pass
    icm426xx_write_reg_SPI(REG_PWR_MGMT0,     0x0F);  // Power ON (Accel + Gyro ON)
    icm426xx_write_reg_SPI(REG_INTF_CONFIG0,  0x20);  // Little Endian format

    ESP_LOGI(TAG_SPI, "ICM-42670-L 3-Wire SPI Initialized.");
    read_attr();

    return ESP_OK;
}