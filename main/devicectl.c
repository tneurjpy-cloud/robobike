/*///////////////////////////////////////////////////////////////////////////////////
    userdevice.c    ESP32 bicycle system
    2026.04.12      Copyright M.Tanaami

    esp32c3 default nvs size=0x5000
///////////////////////////////////////////////////////////////////////////////////*/
#include "userdefine.h"
#include "esp_brownout.h"

static const char TAG[] = "userdevice";
// deviation 偏差

int pcbver = 0;

volatile uint32_t userLastControlTime = 0; // for sleep check counter
volatile uint32_t startTime;
// ADC1のハンドルを保持する変数
adc_oneshot_unit_handle_t adc1_handle;

#define SAVETYPE TSave
static const char *NVSname = "nvs_data";
static volatile bool can_NVS_write = true;

/////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR brownout_callback(void)
{
    can_NVS_write = false;
}

/////////////////////////////////////////////////////////////////////////////////////
/* Standard CRC-32 (Ethernet, PNG, ZIP, etc.) - LSB First */
static uint32_t getCRC32(const uint8_t *p, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < size; i++)
    {
        crc ^= p[i]; /* XOR with the least significant byte */

        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                /* Bit-reflected value of standard polynomial 0x04C11DB7 */
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF; /* Final XOR */
}

static uint32_t getCRC_Tsave(SAVETYPE *data)
{
    uint8_t *p = (uint8_t *)data;
    return getCRC32(p, offsetof(SAVETYPE, CRC));
}

// get system ID = Serial-Code
char *SysID()
{
    static char sisid[9];
    uint8_t mac[6];

    esp_wifi_get_mac(WIFI_IF_AP, mac); // STA or AP
    sprintf(sisid, "%08lX", getCRC32((uint8_t *)mac, sizeof(mac)));
    return sisid;
}

void savenvs()
{
    nvs_handle_t hNVS;

    if (!can_NVS_write)
    {
        return;
    }

    saved.ver = DATAVER;
    saved.op_time_s = startTime + millis() / 1000; // update operation time in seconds
    saved.CRC = getCRC_Tsave(&saved);

    nvs_open(NVSname, NVS_READWRITE, &hNVS);
    esp_err_t err = nvs_set_blob(hNVS, NVSname, &saved, sizeof(saved));
    if (err == ESP_OK)
    {
        nvs_commit(hNVS);
        ESP_LOGI(TAG, "Struct saved successfully!");
    }
    else
    {
        ESP_LOGI(TAG, "Failed to save struct: %s", esp_err_to_name(err));
    }
    nvs_close(hNVS);
}

static bool loadnvs()
{
    nvs_handle_t hNVS;
    uint32_t CRC;
    bool res = false;

    nvs_open(NVSname, NVS_READWRITE, &hNVS);
    size_t sz = sizeof(saved);
    esp_err_t err = nvs_get_blob(hNVS, NVSname, &saved, &sz);
    nvs_close(hNVS);

    if (err == ESP_OK)
    {
        CRC = getCRC_Tsave(&saved);
        if (CRC == saved.CRC)
        {
            if (saved.ver != DATAVER)
            {
                ESP_LOGW(TAG, "Saved DATAVER is not same, load default");
            }
            else
            {
                ESP_LOGI(TAG, "Saved loaded OK!");
                res = true;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Saved CRC ERROR, load default");
        }
    }
    else
    {
        ESP_LOGW(TAG, "No saved data found, load default");
    }

    if (res)
    {
        startTime = saved.op_time_s;
    }
    else
    {
        saved = savedefault;
        startTime = 0;
    }
    return res;
}

void nvs_init()
{
    nvs_handle_t hNVS;

    // 1. NVSの初期化を試みる
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init(); // need initialize again
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS init failed!");
        return;
    }

    err = nvs_open(NVSname, NVS_READWRITE, &hNVS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open failed!");
        return;
    }
    nvs_close(hNVS);

    if (!loadnvs())
    {
        savenvs();
    }
}

////////// Check N ms term ///////////
bool isNms(uint32_t *lastNms, uint32_t Nms)
{
    uint32_t a;

    a = millis();
    if (a - *lastNms >= Nms)
    {
        *lastNms = a;
        return true;
    }
    else
    {
        return false;
    }
}

//// LED PWM using ////////////////////////////////////////////////////////////////
#define LED_CHANNEL LEDC_CHANNEL_3
#define LED_TIMER LEDC_TIMER_1
#define LED_MODE LEDC_LOW_SPEED_MODE
#define LED_FREQUENCY 450               // PWM周波数（Hz）
#define LED_RESOLUTION LEDC_TIMER_8_BIT // 0〜255の輝度制御

static void init_led_pwm(void)
{
    // タイマー設定
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LED_MODE,
        .timer_num = LED_TIMER,
        .duty_resolution = LED_RESOLUTION,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // チャンネル設定
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_GPIO,
        .speed_mode = LED_MODE,
        .channel = LED_CHANNEL,
        .timer_sel = LED_TIMER,
        .duty = 0, // 初期輝度
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}

// brightness: 0〜255
void set_led_brightness(uint8_t brightness)
{
    ledc_set_duty(LED_MODE, LED_CHANNEL, brightness);
    ledc_update_duty(LED_MODE, LED_CHANNEL);
}

uint8_t get_led_brightness()
{
    return ledc_get_duty(LED_MODE, LED_CHANNEL);
}

////////////////////////////////////////////////////////////////////////////////
// wait milisec
// !! Only use in main task !!
void waitms(uint32_t t)
{
    uint32_t start = millis();

    do
    {
        waitTaskms(10);
        if (gy_auto_cal_done)
        { // dim
            uint8_t current_brightness = get_led_brightness();
            set_led_brightness(0);
            waitTaskms(100);
            set_led_brightness(current_brightness);
            gy_auto_cal_done = false;
        }
    } while ((millis() - start) < t);
}

void userdeviceinit()
{
    gpio_config_t io_conf;

    esp_brownout_register_callback(brownout_callback);
    nvs_init(); // nvs memory read

    // ADC1ユニットの初期化設定
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // ADC1_CHANNEL_2 (GPIO 2) の設定
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, // 12bit解像度 (0-4095)
        .atten = ADC_ATTEN_DB_12,    // 約0.1V - 2.8Vの範囲を測定可能
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config);

    // IO0
    io_conf.pin_bit_mask = (1ULL << IO_0);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_0, 1);

    // IO1 FOR autoCircling
    io_conf.pin_bit_mask = (1ULL << IO_1);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_1, 0);

    // IO2 FOR autoCircling
    io_conf.pin_bit_mask = (1ULL << IO_2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // IO20
    io_conf.pin_bit_mask = (1ULL << IO_20);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_20, 0);

    // IO21
    io_conf.pin_bit_mask = (1ULL << IO_21);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_21, 0);

    // LED1  setup
    init_led_pwm();
    set_led_brightness(LEDLOW);
}

//// deep sleep [sec]seconds /////////////////////////////////////////
//// msec: if 0, sleep without wakeup
void deepSleep(uint32_t wup)
{
    ESP_LOGI(TAG, "Sleeping: %ums", wup);

    savenvs();
    stopServo = true;
    waitTaskms(100);
    esp_wifi_stop();
    esp_wifi_deinit();
    IMU_sleep();
    ledc_stop(LED_MODE, LED_CHANNEL, 0); 
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(IO_0, 0);
    gpio_set_level(IO_1, 0);
    gpio_set_level(IO_20, 0);
    gpio_set_level(IO_21, 0);

    if (wup != 0)
    {
        esp_sleep_enable_timer_wakeup((uint64_t)wup * 1000ULL); // usec
    }
    esp_deep_sleep_start();
}
