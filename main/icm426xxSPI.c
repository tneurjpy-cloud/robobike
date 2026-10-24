#include "userdefine.h"

// PCBA v07, ICM-42670-L 3-Wire SPI Driver
#define ICM42670L_WHOAMI 0x63 //
#define PIN_CS IO_9
#define PIN_SCLK IO_8
#define PIN_SDA IO_7

#define REG_DEVICE_CONFIG 0x01 //
#define REG_ACCEL_DATA_X1 0x0B //
#define REG_PWR_MGMT0 0x1F     //
#define REG_GYRO_CONFIG0 0x20  //
#define REG_ACCEL_CONFIG0 0x21 //
#define REG_ACCEL_CONFIG1 0x22 //
#define REG_GYRO_CONFIG1 0x23  // [2,0] GYRO_UI_FILT_BW
#define REG_INTF_CONFIG0 0x35  // for endian change
#define REG_WHO_AM_I 0x75      //

#define IMU_READ_BIT 0x80 //

#define GY_SENSITIVITY (1.0f / 65.5f)
#define GRAVITY 9.80665f
#define AC_SENSITIVITY (GRAVITY / 16384.0f)

#define PACKET_SIZE_BYTES 16  // Packet 3 のサイズ（Header + Accel(6) + Gyro(6) + Temp(1) + TimeStamp(2)）[cite: 28]

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    uint16_t timestamp_us;     // FSYNCからの経過時間（マイクロ秒
    bool is_fsync_synced;      // FSYNC同期フラグ
} imu_packet_data_t;

// ★DMA-compatible buffer (4-byte aligned)
static DMA_ATTR __attribute__((aligned(4))) uint8_t raw[12] = {0};

static const char TAG_SPI[] = "icm426xx_spi";
static spi_device_handle_t spi_handle = NULL;

// Who am I? (SPI version)
static void read_who_am_i()
{
    if (spi_handle == NULL)
        return;

    uint8_t who_am_i = 0;
    spi_transaction_t tx = {
        .flags = SPI_TRANS_USE_RXDATA,        // RXDATA flag is convenient for receptions of 4 bytes or less
        .cmd = (REG_WHO_AM_I | IMU_READ_BIT), // Set read bit
        .length = 0,                          // No transmission data
        .rxlength = 8,                        // Receive 1 byte (8 bits)
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &tx);
    if (ret == ESP_OK)
    {
        who_am_i = tx.rx_data[0]; // Received data is stored in rx_data[0]
        ESP_LOGI(TAG_SPI, "WHO_AM_I: 0x%02x", who_am_i);
    }
    else
    {
        ESP_LOGE(TAG_SPI, "Failed to read WHO_AM_I reg (err: %s)", esp_err_to_name(ret));
    }
}

// Get full-scale sensitivity (SPI version)
// Accel+-  0:16g 1:8g 2:4g 3:2g
// Gyro+-   0:250dps 1:500dps 2:1000dps 3:2000dps
static void icm426xx_get_fs()
{
    if (spi_handle == NULL)
        return;

    uint8_t accel_cfg = 0;
    uint8_t gyro_cfg = 0;

    // 1. Read ACCEL_CONFIG0
    spi_transaction_t tx_accel = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = (REG_ACCEL_CONFIG0 | IMU_READ_BIT),
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &tx_accel) == ESP_OK)
    {
        accel_cfg = (tx_accel.rx_data[0] >> 5) & 0x07;
        ESP_LOGI(TAG_SPI, "Accel FS=%d", accel_cfg);
    }
    else
    {
        ESP_LOGE(TAG_SPI, "Failed to read ACCEL_CONFIG0");
    }

    // 2. Read GYRO_CONFIG0
    spi_transaction_t tx_gyro = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = (REG_GYRO_CONFIG0 | IMU_READ_BIT),
        .length = 0,
        .rxlength = 8,
    };
    if (spi_device_transmit(spi_handle, &tx_gyro) == ESP_OK)
    {
        gyro_cfg = (tx_gyro.rx_data[0] >> 5) & 0x07;
        ESP_LOGI(TAG_SPI, "Gyro FS=%d", gyro_cfg);
    }
    else
    {
        ESP_LOGE(TAG_SPI, "Failed to read GYRO_CONFIG0");
    }
}

// Put ICM-42607-P into sleep mode
void icm426xx_sleep_SPI()
{
    if (spi_handle == NULL)
        return;

    uint8_t sleep_mode = 0x00; // GYRO_MODE=00 (off), ACCEL_MODE=00 (off)
    ESP_LOGI(TAG_SPI, "entering sleep (PWR_MGMT0=0x%02X)", sleep_mode);

    spi_transaction_t tx = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = REG_PWR_MGMT0,     // Do not set the most significant bit (0x80) when writing; specify the address directly
        .tx_data[0] = sleep_mode, // Data to transmit
        .length = 8,              // Transmit 1 byte (8 bits)
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &tx);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI, "Failed to enter sleep mode (err: %s)", esp_err_to_name(ret));
    }
}

// Init ICM-42607-P with 3-wire SPI
esp_err_t icm426xx_init_SPI(void)
{
    // 1. Configure 3-wire half-duplex bus
    static spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_SDA,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32};

    // Configure 3-wire half-duplex interface
    static spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 20 * 1000 * 1000, // MHz
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
        .command_bits = 8,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
    };

    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle));

    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 1: Set bit 2 of DEVICE_CONFIG register to 0 for 3-wire mode
    spi_transaction_t tx_3wire = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = REG_DEVICE_CONFIG, // 0x01
        .tx_data[0] = 0x00,       // Data to enable 3-wire SPI
        .length = 8,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx_3wire));

    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2: Set initial configuration values using defined macros
    uint8_t init_cmds[][2] = {
        {REG_INTF_CONFIG0, 0x20},  // 0x35: Change to Little Endian format
        {REG_GYRO_CONFIG0, 0x45},  // 0x20: 500dps, 1.6kHz
        {REG_ACCEL_CONFIG0, 0x65}, // 0x21: 0b01100101
        {REG_GYRO_CONFIG1, 0x00},  // 0x23: bypass Lo-pass (0x00)
        {REG_ACCEL_CONFIG1, 0x07}, // 0x22: 40Hz Lo-pass (0x07)
        {REG_PWR_MGMT0, 0x0F},     // 0x1F: Turn on accel & gyro in LN mode
    };

    for (int i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++)
    {
        spi_transaction_t tx = {
            .flags = SPI_TRANS_USE_TXDATA,
            .cmd = init_cmds[i][0],
            .tx_data[0] = init_cmds[i][1],
            .length = 8,
        };
        ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    }

    ESP_LOGI(TAG_SPI, "ICM-42607-P 3-Wire SPI Initialized.");
    read_who_am_i();
    icm426xx_get_fs();

    return ESP_OK;
}

// Dummy function
void icm426xx_start_read_SPI()
{
    // only dummy function for SPI, because SPI read is done in icm426xx_get_data_SPI()
}

// CS length = 5.26us @20MHz
void icm426xx_get_data_SPI(Tvector6d *pac)
{
    if (spi_handle == NULL)
        return;

    static spi_transaction_t rx = {
        .cmd = (REG_ACCEL_DATA_X1 | IMU_READ_BIT),
        .rx_buffer = raw, // DMA buffer
        .length = 0,
        .rxlength = 12 * 8, // receiving data length = 12 bytes
    };

    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &rx));

    pac->x = ((int16_t *)raw)[0] * AC_SENSITIVITY;
    pac->y = ((int16_t *)raw)[1] * AC_SENSITIVITY;
    pac->z = ((int16_t *)raw)[2] * AC_SENSITIVITY;
    pac->gx = ((int16_t *)raw)[3] * GY_SENSITIVITY;
    pac->gy = ((int16_t *)raw)[4] * GY_SENSITIVITY;
    pac->gz = ((int16_t *)raw)[5] * GY_SENSITIVITY;
}

/////////////////////////////////////////////////////////////////////////////////////

// MREG1間接書き込み用ヘルパー関数
static void icm426xx_write_mreg1(uint8_t reg_addr, uint8_t data)
{
    if (spi_handle == NULL) return;
    
    spi_transaction_t tx_sel = { .flags = SPI_TRANS_USE_TXDATA, .cmd = 0x79, .tx_data[0] = 0x00, .length = 8 };
    spi_device_transmit(spi_handle, &tx_sel);
    spi_transaction_t tx_addr = { .flags = SPI_TRANS_USE_TXDATA, .cmd = 0x7A, .tx_data[0] = reg_addr, .length = 8 };
    spi_device_transmit(spi_handle, &tx_addr);
    spi_transaction_t tx_val = { .flags = SPI_TRANS_USE_TXDATA, .cmd = 0x7B, .tx_data[0] = data, .length = 8 };
    spi_device_transmit(spi_handle, &tx_val);
    
    esp_rom_delay_us(10);

    spi_transaction_t tx_reset = { .flags = SPI_TRANS_USE_TXDATA, .cmd = 0x79, .tx_data[0] = 0x00, .length = 8 };
    spi_device_transmit(spi_handle, &tx_reset);
}

// FIFOおよびFSYNCタイムスタンプの初期設定
void icm426xx_fifo_init(void)
{
    icm426xx_write_mreg1(0x00, 0x03); // TMST_EN=1, TMST_FSYNC_EN=1[cite: 74]
    icm426xx_write_mreg1(0x01, 0x07); // FIFO_ACCEL_EN=1, FIFO_GYRO_EN=1, FIFO_TMST_FSYNC_EN=1[cite: 75]
}

// FIFOから蓄積データを読み出し、最新の1セットのみをデコードして取得する関数
static uint8_t fifo_raw_buf[512] = {0};
bool icm426xx_read_latest_fifo_packet(imu_packet_data_t *packet)
{
    if (spi_handle == NULL) return false;

    // 1. FIFOのデータバイト数を取得 (FIFO_COUNTH: 0x3D, FIFO_COUNTL: 0x3E)
    uint8_t count_bytes[2] = {0};
    spi_transaction_t tx_count = {
        .cmd = (0x3D | IMU_READ_BIT),
        .rx_buffer = count_bytes,
        .length = 0,
        .rxlength = 2 * 8,
    };

    if (spi_device_transmit(spi_handle, &tx_count) != ESP_OK) {
        return false;
    }

    uint16_t fifo_count = ((uint16_t)count_bytes[0] << 8) | count_bytes[1];
    if (fifo_count < PACKET_SIZE_BYTES) {
        return false; // 1パケット分も溜まっていない
    }

    // バッファ溢れ防止のキャップ（必要に応じて調整）
    if (fifo_count > 512) fifo_count = 512;

    // 2. FIFOに溜まった分をすべて読み出してクリアする
    spi_transaction_t rx_burst = {
        .cmd = (0x3F | IMU_READ_BIT), // FIFO_DATA (0x3F)
        .rx_buffer = fifo_raw_buf,
        .length = 0,
        .rxlength = fifo_count * 8,
    };

    if (spi_device_transmit(spi_handle, &rx_burst) != ESP_OK) {
        return false;
    }

    // 3. 取得したデータ群のうち、一番最後（＝最も新しい）のパケットの位置を特定
    uint16_t num_packets = fifo_count / PACKET_SIZE_BYTES;
    if (num_packets == 0) return false;

    uint8_t *p = &fifo_raw_buf[(num_packets - 1) * PACKET_SIZE_BYTES];

    // 4. 最新パケットのみを解析・デコード
    uint8_t header = p[0];
    if (header & 0x80) {
        return false; // 無効パケット
    }

    uint8_t ts_fsync_bits = (header >> 2) & 0x03;
    packet->is_fsync_synced = (ts_fsync_bits >= 2);

    int16_t raw_ax = ((int16_t)p[1] << 8) | p[2];
    int16_t raw_ay = ((int16_t)p[3] << 8) | p[4];
    int16_t raw_az = ((int16_t)p[5] << 8) | p[6];

    int16_t raw_gx = ((int16_t)p[7] << 8) | p[8];
    int16_t raw_gy = ((int16_t)p[9] << 8) | p[10];
    int16_t raw_gz = ((int16_t)p[11] << 8) | p[12];

    packet->timestamp_us = ((uint16_t)p[14] << 8) | p[15]; // FSYNCからの経過時間

    packet->ax = (float)raw_ax * AC_SENSITIVITY;
    packet->ay = (float)raw_ay * AC_SENSITIVITY;
    packet->az = (float)raw_az * AC_SENSITIVITY;
    packet->gx = (float)raw_gx * GY_SENSITIVITY;
    packet->gy = (float)raw_gy * GY_SENSITIVITY;
    packet->gz = (float)raw_gz * GY_SENSITIVITY;

    return true;
}
