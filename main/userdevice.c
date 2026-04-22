/*///////////////////////////////////////////////////////////////////////////////////
    userdevice.c    ESP32 bicycle system
    2026.04.12      Copyright M.Tanaami

    esp32c3 default nvs size=0x5000
///////////////////////////////////////////////////////////////////////////////////*/
#include "userdefine.h"

static const char TAG[] = "userdevice";
// deviation 偏差

int pcbver = 0;

volatile uint32_t millis01 = 0;            // 0.1msec counter
volatile uint32_t millis = 0;              // 1msec counter
volatile uint32_t userLastControlTime = 0; // for sleep check counter

// static char *NVSserial = "nvs_serial";
#define SAVETYPE TSave
static const char *NVSname = "nvs_data";

// 0.1msec timer
static gptimer_handle_t timerMsec01;

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

    saved.ver = DATAVER;
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
                saved = savedefault;
                return false;
            }
            else
            {
                ESP_LOGI(TAG, "Saved loaded OK!");
                return true;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Saved CRC ERROR, load default");
            saved = savedefault;
            return false;
        }
    }
    else
    {
        ESP_LOGW(TAG, "No saved data found, load default");
        saved = savedefault; // デフォルト値を設定
        return false;
    }
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

//// deep sleep [sec]seconds /////////////////////////////////////////
//// msec: if 0, sleep without wakeup

void deepSleep(uint32_t ms)
{
    ESP_LOGI(TAG, "Sleeping: %dms", (int)ms);

    gpio_set_level(IO_SV_EN, 0);
    IMU_sleep();
    // gptimer_stop(timerMsec01);
    // gptimer_disable(timerMsec01);

    if (ms != 0)
    {
        esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000); // usec
    }
    esp_deep_sleep_start();
}

////////// Check N ms term ///////////
bool isNms(uint32_t *lastNms, uint32_t Nms)
{
    uint32_t a;

    a = millis;
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

//// Only count up the millis counter //////////////////////////////////////////
// - ESP32の割り込みコントローラは、割り込みが処理されないまま次の周期が来ると、
// 実行が1回スキップされる
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    millis01++;
    if (millis01 % 10 == 0) // see alarm_count, resolution_hz
        millis++;
    return false; // No context switch required
}

static void setup_timer_interrupt()
{
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 tick = 1μs
    };
    gptimer_new_timer(&config, &timerMsec01);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm,
    };
    gptimer_register_event_callbacks(timerMsec01, &cbs, NULL);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 100, // 100μs
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(timerMsec01, &alarm_config);

    gptimer_enable(timerMsec01);
    gptimer_start(timerMsec01);
}

//// LED PWM using ////////////////////////////////////////////////////////////////
#define LED_GPIO IO_LED1 // 接続するGPIO番号
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

void set_led_brightness(uint8_t brightness)
{
    // brightness: 0〜255
    ledc_set_duty(LED_MODE, LED_CHANNEL, brightness);
    ledc_update_duty(LED_MODE, LED_CHANNEL);
}

////////////////////////////////////////////////////////////////////////////////
// wait milisec
// CONFIG_FREERTOS_HZ=1000   in "sdkconfig" for 1ms delay
void waitms(uint32_t t)
{
    uint32_t start = millis;

    do
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (gy_auto_cal_done)
        { // dim
            set_led_brightness(0);
            vTaskDelay(pdMS_TO_TICKS(100));
            set_led_brightness(LEDLOW);
            gy_auto_cal_done = false;
        }
    } while ((millis - start) < t);
}

void userdeviceinit()
{
    void servo_init();
    gpio_config_t io_conf;

    nvs_init();              // nvs memory read
    setup_timer_interrupt(); // 1msec timer interrupt

    // IO10  setup
    io_conf.pin_bit_mask = (1ULL << IO_10);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_10, 0);

    // TP1  setup
    io_conf.pin_bit_mask = (1ULL << IO_0);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_0, 0);

    // TP2  setup
    io_conf.pin_bit_mask = (1ULL << IO_1);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_1, 0);

    // TP3  setup
    io_conf.pin_bit_mask = (1ULL << IO_2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // IO_SV_EN  setup
    io_conf.pin_bit_mask = (1ULL << IO_SV_EN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_SV_EN, 1); // Power of Servo ON

    // IO20  setup
    io_conf.pin_bit_mask = (1ULL << IO_20);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_20, 0);

    // IO21  setup
    io_conf.pin_bit_mask = (1ULL << IO_21);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(IO_21, 0);

    IMU_init();
    servo_init();

    // LED1  setup
    init_led_pwm();
    set_led_brightness(LEDLOW);
}