#include "userdefine.h"

static const char TAG[] = "icm426xx_spi";

// uint16_t imu_timestamp_us = 0;
volatile uint16_t fsync_time_delta = 0; // FSYNCからODRまでのタイムデルタ

#define ICM42670L_WHOAMI 0x63
#define PIN_CS IO_9
#define PIN_SCLK IO_8
#define PIN_SDA IO_7

// Bank 0 レジスタ定義
#define REG_DEVICE_CONFIG 0x01
#define REG_ACCEL_DATA_X1 0x0B
#define REG_TMST_FSYNCH 0x17 // FSYNCタイムデルタ Hiバイト (0x17)
#define REG_TMST_FSYNCL 0x18 // FSYNCタイムデルタ Loバイト (0x18)
#define REG_PWR_MGMT0 0x1F
#define REG_GYRO_CONFIG0 0x20
#define REG_ACCEL_CONFIG0 0x21
#define REG_ACCEL_CONFIG1 0x22
#define REG_GYRO_CONFIG1 0x23
#define REG_INTF_CONFIG0 0x35
#define REG_WHO_AM_I 0x75

// Bank MREG1 アクセス用（Bank 0 レジスタ）
#define REG_BLK_SEL_W 0x79
#define REG_MADDR_W 0x7A
#define REG_M_W 0x7B
#define REG_BLK_SEL_R 0x7C
#define REG_MADDR_R 0x7D
#define REG_M_R 0x7E

#define IMU_READ_BIT 0x80

#define GY_SENSITIVITY (1.0f / 65.5f)       // deg/sec/LSB  ±500 dps
#define GRAVITY 9.80665f                    //
#define AC_SENSITIVITY (GRAVITY / 16384.0f) // ±2g設定時 (m/s^2 単位)

static spi_device_handle_t spi_handle = NULL;

//////////////////////////////////////////////////////////////////////////////
// 書き込み関数（command_bits = 8 を使用）
static esp_err_t icm426xx_write_reg_SPI(uint8_t reg, uint8_t val)
{
    if (spi_handle == NULL)
        return ESP_FAIL;

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

// MREG1 間接書き込み用関数
static esp_err_t icm426xx_write_mreg1_SPI(uint8_t reg, uint8_t val)
{
    esp_err_t ret;
    ret = icm426xx_write_reg_SPI(REG_BLK_SEL_W, 0x00); // MREG1選択
    if (ret != ESP_OK)
        return ret;
    ret = icm426xx_write_reg_SPI(REG_MADDR_W, reg);
    if (ret != ESP_OK)
        return ret;
    ret = icm426xx_write_reg_SPI(REG_M_W, val);
    if (ret != ESP_OK)
        return ret;

    esp_rom_delay_us(10); // MREGアクセス後の必要な10usウェイト

    // BLK_SEL_W を0x00にリセット
    return icm426xx_write_reg_SPI(REG_BLK_SEL_W, 0x00);
}

// MREG1 間接読み出し用関数
static uint8_t icm426xx_read_mreg1_SPI(uint8_t reg)
{
    uint8_t val = 0;
    icm426xx_write_reg_SPI(REG_BLK_SEL_R, 0x00); // MREG1選択 (Read用)
    icm426xx_write_reg_SPI(REG_MADDR_R, reg);
    esp_rom_delay_us(10);

    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = REG_M_R | IMU_READ_BIT,
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &t) == ESP_OK)
    {
        val = t.rx_data[0];
    }
    esp_rom_delay_us(10);
    icm426xx_write_reg_SPI(REG_BLK_SEL_R, 0x00);
    return val;
}

// 属性およびMREG1設定の確認用関数
static void read_attr()
{
    if (spi_handle == NULL)
        return;

    // 1. WHO_AM_I (0x75) の読み出し
    spi_transaction_t t_whoami = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = REG_WHO_AM_I | IMU_READ_BIT,
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &t_whoami) == ESP_OK)
    {
        ESP_LOGI(TAG, "WHO_AM_I: 0x%02x", t_whoami.rx_data[0]);
    }

    // 2. GYRO_CONFIG0 (0x20) の読み出し（FS と ODR）
    spi_transaction_t t_gyro_cfg = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = REG_GYRO_CONFIG0 | IMU_READ_BIT,
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &t_gyro_cfg) == ESP_OK)
    {
        uint8_t val = t_gyro_cfg.rx_data[0];
        uint8_t fs_sel = (val >> 5) & 0x03; // [6:5]
        uint8_t odr = val & 0x0F;           // [3:0]
        ESP_LOGI(TAG, "GYRO_CONFIG0: 0x%02x (FS_SEL=%d, ODR=%d)", val, fs_sel, odr);
    }

    // 3. ACCEL_CONFIG0 (0x21) の読み出し（FS と ODR）
    spi_transaction_t t_accel_cfg = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = REG_ACCEL_CONFIG0 | IMU_READ_BIT,
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &t_accel_cfg) == ESP_OK)
    {
        uint8_t val = t_accel_cfg.rx_data[0];
        uint8_t fs_sel = (val >> 5) & 0x03; // [6:5]
        uint8_t odr = val & 0x0F;           // [3:0]
        ESP_LOGI(TAG, "ACCEL_CONFIG0: 0x%02x (FS_SEL=%d, ODR=%d)", val, fs_sel, odr);
    }

    // 4. MREG1: TMST_CONFIG1 (アドレス 0x00) の読み出し確認
    uint8_t tmst_cfg1 = icm426xx_read_mreg1_SPI(0x00);
    ESP_LOGI(TAG, "MREG1 TMST_CONFIG1: 0x%02x", tmst_cfg1);
}

void icm426xx_sleep_SPI()
{
    if (spi_handle == NULL)
        return;

    ESP_LOGI(TAG, "entering sleep (PWR_MGMT0=0x00)");
    icm426xx_write_reg_SPI(REG_PWR_MGMT0, 0x00);
}

////////// dummy
void icm426xx_start_read_SPI()
{
}

// Initialize timestamp
static void init_FSYNC()
{
    // Step 3: Configure FSYNC / timestamp function (MREG1: TMST_CONFIG1 = 0x07)
    // Bit 0: TMST_EN = 1 (Enable timestamp)
    // Bit 1: TMST_FSYNC_EN = 1 (Enable FSYNC timestamp)
    // Bit 2: TMST_DELTA_EN = 1 (Enable timestamp delta)
    icm426xx_write_mreg1_SPI(0x00, 0x07);

    icm426xx_write_reg_SPI(REG_PWR_MGMT0, 0x0F);    // Power ON (Accel + Gyro ON)
    icm426xx_write_reg_SPI(REG_INTF_CONFIG0, 0x20); // Little Endian format
}

// Burst read 14 bytes: Accel & Gyro (12 bytes) + FSYNC time delta (2 bytes)
void icm426xx_get_data_SPI()
{
    if (spi_handle == NULL)
        return;

    // Allocate 14 bytes buffer from 0x0B (ACCEL_DATA_X1) to 0x18 (TMST_FSYNCL)
    uint8_t raw[14] = {0};

    spi_transaction_t t_burst = {
        .flags = 0,
        .cmd = REG_ACCEL_DATA_X1 | IMU_READ_BIT,
        .length = 0,        // No TX data (command phase only)
        .rxlength = 14 * 8, // RX data length: 14 bytes = 112 bits
        .rx_buffer = raw,
        .tx_buffer = NULL,
    };

    if (spi_device_transmit(spi_handle, &t_burst) != ESP_OK)
    {
        return;
    }

    // Process sensor data (little-endian format)
    // raw[0..11] contains Accel (X,Y,Z) and Gyro (X,Y,Z)
    int16_t *raw_16 = (int16_t *)raw;
    acc.x = raw_16[0] * AC_SENSITIVITY;
    acc.y = raw_16[1] * AC_SENSITIVITY;
    acc.z = raw_16[2] * AC_SENSITIVITY;
    acc.gx = raw_16[3] * GY_SENSITIVITY;
    acc.gy = raw_16[4] * GY_SENSITIVITY;
    acc.gz = raw_16[5] * GY_SENSITIVITY;
    fsync_time_delta = *(uint16_t *)(&raw[12]); // Time from FSYNC to the first sample in us
}

esp_err_t icm426xx_init_SPI()
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
    icm426xx_write_reg_SPI(REG_GYRO_CONFIG0, 0x45);  // 500dps, 1.6kHz
    icm426xx_write_reg_SPI(REG_ACCEL_CONFIG0, 0x65); // ±2g, 1.6kHz
    icm426xx_write_reg_SPI(REG_GYRO_CONFIG1, 0x00);  // bypass Lo-pass
    icm426xx_write_reg_SPI(REG_ACCEL_CONFIG1, 0x07); // 40Hz Lo-pass

    init_FSYNC();
    ESP_LOGI(TAG, "ICM-42670-L 3-Wire SPI Initialized with FSYNC Timestamp.");
    init_FSYNC(); // OMJINAI for PON RESET
    read_attr();

    return ESP_OK;
}