/*
CmakeList.txt は下記とする
idf_component_register(SRCS main.c servo.c IMU.c icm426xx.c userdevice.c webserver.c
   EMBED_FILES root.html setup.html setup2.html ota.html)
*/
#include "userdefine.h"
#include <esp_http_server.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

static const char *TAG = "websrv";

//// web page file resources ////
extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");
extern const int32_t root_len asm("root_html_length");

extern const char setup_start[] asm("_binary_setup_html_start");
extern const char setup_end[] asm("_binary_setup_html_end");
extern const int32_t setup_len asm("setup_html_length");

extern const char setup2_start[] asm("_binary_setup2_html_start");
extern const char setup2_end[] asm("_binary_setup2_html_end");
extern const int32_t setup2_len asm("setup2_html_length");

extern const char ota_start[] asm("_binary_ota_html_start");
extern const char ota_end[] asm("_binary_ota_html_end");
extern const int32_t ota_len asm("ota_html_length");

extern const char monitor_start[] asm("_binary_monitor_html_start");
extern const char monitor_end[] asm("_binary_monitor_html_end");
extern const int32_t monitor_len asm("monitor_html_length");

extern const char favicon_start[] asm("_binary_favicon_ico_start");
extern const char favicon_end[] asm("_binary_favicon_ico_end");
extern const int32_t favicon_len asm("favicon_ico_length");

static httpd_handle_t http_server = NULL;

static char *mkcsv()
{
    static char rescsv[128];
    memset(rescsv, '\0', sizeof(rescsv));

    float str_cmd_rate = (float)str_cmd0 / (float)saved.str_turn * STR_SLIDER_MAX;

    /*  for java script
    const CSV_KEYS = [
        "HEADER",       // 0: 'b'
        "PROG_VER",     // 1
        "DATA_VER",     // 2
        "STATUS",       // 3
        "STR0",         // 4
        "MOT_SPD",      // 5
        "GAIN_STR",     // 6
        "GAIN_W_ROLL",  // 7
        "GAIN_DIFF",    // 8
        "ANG_STD_NUT",  // 9
        "STR_TURN",     // 10
        "YAW_COEFF",    // 11
        "AUTO_CIRCLING",// 12
        "STR_CMD_RATE"  // 13
    ];
    */
    snprintf(rescsv, sizeof(rescsv),
             "b,%d,%d,%s,%d,%d,%.3f,%.3f,%.3f,%d,%d,%.3f,%d,%.3f",
             PROGVER,
             DATAVER,
             "OK",
             saved.str0,
             saved.mot_spd,
             saved.gain_str,
             saved.gain_w_roll,
             saved.gain_str_diff,
             saved.ang_std_nut,
             saved.str_turn,
             saved.yaw_coeff,
             autoCircling,
             str_cmd_rate);

    ESP_LOGI(TAG, "length=%d\ncsv=%s", strlen(rescsv), rescsv);

    return rescsv;
}

/////////////////////////////////////////////////////////////////////////////
// HTTP GET Handler
/// @brief  "http://192.168.4.1/"
/// @param req
/// @return
#include <lwip/sockets.h>
#include <arpa/inet.h>

static struct in_addr master_ip = {.s_addr = 0};

static esp_err_t root_get_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (fd < 0)
    {
        ESP_LOGE(TAG, "Invalid socket fd");
        return ESP_FAIL;
    }

    struct sockaddr_storage addr;
    socklen_t addr_len = sizeof(addr);
    memset(&addr, 0, sizeof(addr));

    char ip_str[INET6_ADDRSTRLEN] = "0.0.0.0";
    uint16_t port = 0;
    struct in_addr client_ip = {.s_addr = 0};

    if (getpeername(fd, (struct sockaddr *)&addr, &addr_len) != 0)
    {
        ESP_LOGE(TAG, "getpeername failed errno=%d", errno);
    }
    else
    {
        if (addr.ss_family == AF_INET)
        {
            // ===== IPv4 =====
            struct sockaddr_in *a = (struct sockaddr_in *)&addr;
            client_ip = a->sin_addr;
            inet_ntop(AF_INET, &a->sin_addr, ip_str, sizeof(ip_str));
            port = ntohs(a->sin_port);
        }
        else if (addr.ss_family == AF_INET6)
        {
            // ===== IPv6 =====
            struct sockaddr_in6 *a6 = (struct sockaddr_in6 *)&addr;
            inet_ntop(AF_INET6, &a6->sin6_addr, ip_str, sizeof(ip_str));
            port = ntohs(a6->sin6_port);

            // ★ IPv4-mapped IPv6 判定
            if (IN6_IS_ADDR_V4MAPPED(&a6->sin6_addr))
            {
                // ::FFFF:xxxx:xxxx → IPv4抽出
                memcpy(&client_ip.s_addr,
                       &a6->sin6_addr.s6_addr[12],
                       sizeof(client_ip.s_addr));

                char ipv4_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_ip, ipv4_str, sizeof(ipv4_str));

                ESP_LOGI(TAG, "IPv4 extracted from IPv6: %s", ipv4_str);
            }
            else
            {
                client_ip.s_addr = 0; // 純IPv6は今回は無視
            }
        }
    }

    ESP_LOGI(TAG, "Request [FD:%d] [IP:%s] [Port:%d]", fd, ip_str, port);

    // ===== 初期設定チェック =====
    if (!saved.isChecked)
    {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/setup");
        return httpd_resp_send(req, NULL, 0);
    }

    // ===== Master登録 =====
    if (client_ip.s_addr != 0 && master_ip.s_addr == 0)
    {
        master_ip = client_ip;

        char master_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &master_ip, master_str, sizeof(master_str));

        ESP_LOGI(TAG, "★★★ MASTER REGISTERED: %s ★★★", master_str);
    }

    // ===== レスポンス =====
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

    if (client_ip.s_addr != 0 && client_ip.s_addr == master_ip.s_addr)
    {
        ESP_LOGI(TAG, "Serving MASTER page");
        return httpd_resp_send(req, root_start, root_len);
    }
    else
    {
        ESP_LOGI(TAG, "Serving GUEST page");
        return httpd_resp_send(req, monitor_start, monitor_len);
    }
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler};

#define IF_BTEQ(str) if (strcmp(bt_name, str) == 0)
#define ELSE_IF_BTEQ(str) else if (strcmp(bt_name, str) == 0)

/// TASK "httpd" ///////////////////////////////////////////
/// @brief  "http://192.168.4.1/command?button=bt_S"
static esp_err_t command_handler(httpd_req_t *req)
{
    char bt_name[64];
    esp_err_t err;
    bool ret_data = true;
    bool isControl = false;

    ESP_LOGI(TAG, "URI: %s", req->uri);
    err = httpd_req_get_url_query_str(req, bt_name, sizeof(bt_name));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "q_str err=%s", esp_err_to_name(err));
        return err;
    }
    err = httpd_query_key_value(bt_name, "button", bt_name, sizeof(bt_name));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "q_key err=%s", esp_err_to_name(err));
        return err;
    }

    ///////////////////////////////////////////////////////
    isControl = true;
    IF_BTEQ("stp_all") // return from setup to root
    {
        auto_disable();
        set_mot_duty(0.0f, SERVO_NEUTRAL_DUTY);
        set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, SERVO_NEUTRAL_DUTY);
        set_str_cmd(0, SERVO_NEUTRAL_DUTY);
        savenvs(); // save NVS flash memory
    }
    ELSE_IF_BTEQ("only_data")
    {
    }
    ELSE_IF_BTEQ("bt_F") // start or go straight
    {
        if (mot_out != 0.0f) // 走行中
        {
            set_str_cmd(0.0f, STR_CMD_SPD_N);
        }
        else // start on stop
        {    // down the stand slowly, motor on, up the stand
            auto_disable();
            set_led_brightness(LEDHIGH);
            set_str_cmd(0.0f, SERVO_NEUTRAL_DUTY);
            set_ex1_angle((float)((STD_STD_NUT + saved.ang_std_nut) + saved.ang_std_nut * 3) / 4, 1.0f);
            wait_ex1_angle();
            set_ex1_angle(saved.ang_std_nut, 0.1f);
            wait_ex1_angle();
            set_mot_duty(saved.mot_spd, 50.f);
            auto_enable();
            set_ex1_angle(STD_RUN, 10.f);
        }
    }
    ELSE_IF_BTEQ("bt_S") // go straight, tilt left and stop
    {
        if (mot_out > 0)
        {
            if (str_cmd2 != 0.f)
            {
                set_str_cmd(0.0f, STR_CMD_SPD_N * 2.0f);
                wait_str_angle();
            }
            set_str_cmd(-STR_STOP, 10.0f);                                       // 左傾を誘発
            set_ex1_angle(saved.ang_std_nut + STD_STD_NUT, SERVO_NEUTRAL_DUTY); // スタンドを先に出す
            vTaskDelay(pdMS_TO_TICKS(400));
            wait_str_angle();
            auto_disable();
            set_str_cmd(STR_STOP, 0.0f);
            set_mot_duty(0.0f, 4.0f);
            wait_mot_duty();
            set_led_brightness(LEDLOW);
        }
        else
        {
            set_mot_duty(0.0f, 0.0f);
        }
        set_str_cmd(0.0f, 0.0f);
    }
    ELSE_IF_BTEQ("bt_L")
    {
        set_str_cmd(-saved.str_turn, STR_CMD_SPD_P);
    }
    ELSE_IF_BTEQ("bt_R")
    {
        set_str_cmd(saved.str_turn, STR_CMD_SPD_P);
    }
    ELSE_IF_BTEQ("bt_Str_S")
    {
        char query_str[64];
        char value_str[64];

        err = httpd_req_get_url_query_str(req, query_str, sizeof(query_str));
        err = httpd_query_key_value(query_str, "value", value_str, sizeof(value_str));
        float value = atof(value_str);
        set_str_cmd(value * saved.str_turn / STR_SLIDER_MAX, STR_CMD_SPD_P);
    }
    ELSE_IF_BTEQ("bt_BK") // go backward/stop
    {
        if (mot_out == 0) // 停止中
        {
            set_mot_duty(MOT_SPEED_BACK, 0.0f);
        }
        else if (mot_out < 0)
        {
            set_mot_duty(0.0f, 0.0f);
        }
    }
    //////// Parameters tuning in root.html /////////
    ELSE_IF_BTEQ("bt_A_Up_0") // mot speed up
    {
        if (saved.mot_spd < MOTMAX)
            saved.mot_spd++;
        if (mot_out > 0) // 走行中
        {
            set_mot_duty(saved.mot_spd, 0.0f);
        }
    }
    ELSE_IF_BTEQ("bt_A_Dn_0") // mot speed dn
    {
        if (saved.mot_spd > 0)
            saved.mot_spd--;
        if (mot_out > 0) // 走行中
        {
            set_mot_duty(saved.mot_spd, 0.0f);
        }
    }
    //////// Parameters tuning in setup.html /////////
    ELSE_IF_BTEQ("bt_A_Up") // mot speed up
    {
        if (saved.mot_spd < MOTMAX)
            saved.mot_spd++;
        set_mot_duty(saved.mot_spd, 0.0f);
    }
    ELSE_IF_BTEQ("bt_A_Dn") // mot speed dn
    {
        if (saved.mot_spd > 0)
            saved.mot_spd--;
        set_mot_duty(saved.mot_spd, 0.0f);
    }
    ELSE_IF_BTEQ("bt_A_L") // str nut >> L
    {
        if (saved.str0 > STR_ADJ_MIN)
            saved.str0--;
        set_str_cmd(0.0f, 0.0f); // + = right turn
        set_mot_duty(0.0f, 0.0f);
    }
    ELSE_IF_BTEQ("bt_A_R") // str nut >> R
    {
        if (saved.str0 < STR_ADJ_MAX)
            saved.str0++;
        set_str_cmd(0.0f, 0.0f); // + = right turn
        set_mot_duty(0.0f, 0.0f);
    }
    ELSE_IF_BTEQ("bt_S_Up") // str gain
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str < STR_GA_MAX)
            saved.gain_str = ((int)(saved.gain_str * 1000) + 1) / 1000.f;
    }
    ELSE_IF_BTEQ("bt_S_Dn") //
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str > STR_GA_MIN)
            saved.gain_str = ((int)(saved.gain_str * 1000) - 1) / 1000.f;
    }
    ELSE_IF_BTEQ("bt_R_Up") // roll gain
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_w_roll < ROLL_GA_MAX)
            saved.gain_w_roll = ((int)(saved.gain_w_roll * 1) + 1);
    }
    ELSE_IF_BTEQ("bt_R_Dn") //
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_w_roll > ROLL_GA_MIN)
            saved.gain_w_roll = ((int)(saved.gain_w_roll * 1) - 1);
    }
    ELSE_IF_BTEQ("bt_D_St")
    {
        set_mot_duty(0.0f, 0.0f);
    }
    ELSE_IF_BTEQ("bt_D_Up") // diff gain
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str_diff < DIFF_GA_MAX)
            saved.gain_str_diff = ((int)(saved.gain_str_diff * 1000) + 1) / 1000.f;
    }
    ELSE_IF_BTEQ("bt_D_Dn") //
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.gain_str_diff > DIFF_GA_MIN)
            saved.gain_str_diff = ((int)(saved.gain_str_diff * 1000) - 1) / 1000.f;
    }
    ELSE_IF_BTEQ("bt_Std_nutAuto") // Side stand adjustment
    {
        set_mot_duty(0.0f, 0.0f);
        set_ex1_angle(-10.0f, 0.1f);
        while (ex1_out > ex1_cmd)
        {
            ESP_LOGI(TAG, "az=%f", LATERAL_G);
            vTaskDelay(pdMS_TO_TICKS(25));
            if (ex1_out <= ex1_cmd)
            {
                break;
            }
            else if (LATERAL_G <= 0.5f)
            {
                vTaskDelay(pdMS_TO_TICKS(45));
                if (LATERAL_G <= 0.5f)
                {
                    break;
                }
            }
        }

        if (saved.ang_std_nut < EX1MAX)
            saved.ang_std_nut += 1;

        set_ex1_angle(saved.ang_std_nut, 1);
    }
    ELSE_IF_BTEQ("bt_Std_nutUp")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.ang_std_nut < EX1MAX)
            saved.ang_std_nut += 1;
        set_ex1_angle(saved.ang_std_nut, 1);
    }
    ELSE_IF_BTEQ("bt_Std_nutDn")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.ang_std_nut > EX1MIN)
            saved.ang_std_nut -= 1;
        set_ex1_angle(saved.ang_std_nut, 1);
    }
    ELSE_IF_BTEQ("bt_str_turnUp")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_turn < STRMAX)
            saved.str_turn += 1;
    }
    ELSE_IF_BTEQ("bt_str_turnDn")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.str_turn > (STRMAX / 10))
            saved.str_turn -= 1;
    }
    ELSE_IF_BTEQ("IR_ON")
    {
        set_mot_duty(0.0f, 0.0f);
        autoCircling = true;
    }
    ELSE_IF_BTEQ("IR_OFF")
    {
        set_mot_duty(0.0f, 0.0f);
        autoCircling = false;
    }
    ELSE_IF_BTEQ("bt_yaw_coeffUp")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.yaw_coeff < GYDIR_YAW_MAX)
            saved.yaw_coeff += 0.001f;
        if (saved.yaw_coeff > GYDIR_YAW_MAX)
            saved.yaw_coeff = GYDIR_YAW_MAX;
    }
    ELSE_IF_BTEQ("bt_yaw_coeffDn")
    {
        set_mot_duty(0.0f, 0.0f);
        if (saved.yaw_coeff > GYDIR_YAW_MIN)
            saved.yaw_coeff -= 0.001f;
        if (saved.yaw_coeff < GYDIR_YAW_MIN)
            saved.yaw_coeff -= GYDIR_YAW_MIN;
    }
    ELSE_IF_BTEQ("bt_Ld_Default")
    {
        set_mot_duty(0.0f, 0.0f);
        saved = savedefault;
    }
    else
    {
        isControl = false;
        ret_data = false;
    }

    //////// Control running /////////
    if (isControl)
        userLastControlTime = millis;

    httpd_resp_set_type(req, "text/plain");
    if (ret_data)
    { // YOU MUST DO httpd_resp_send()
        httpd_resp_send(req, mkcsv(), HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static const httpd_uri_t command = {
    .uri = "/command",
    .method = HTTP_GET,
    .handler = command_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/setup"
/// @param req
/// @return
static esp_err_t setup_get_handler(httpd_req_t *req)
{
    // const uint32_t L = setup_end - setup_start;

    ESP_LOGI(TAG, "Serve setup");
    saved.isChecked = true;
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_send(req, setup_start, setup_len);
    auto_disable(); // stop automated control

    return ESP_OK;
}

static const httpd_uri_t setup = {
    .uri = "/setup",
    .method = HTTP_GET,
    .handler = setup_get_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/setup2"
/// @param req
/// @return
static esp_err_t setup2_get_handler(httpd_req_t *req)
{
    // const uint32_t L = setup_end - setup_start;

    ESP_LOGI(TAG, "Serve setup2");
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_send(req, setup2_start, setup2_len);
    auto_disable(); // stop automated control

    return ESP_OK;
}

static const httpd_uri_t setup2 = {
    .uri = "/setup2",
    .method = HTTP_GET,
    .handler = setup2_get_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/monitor"
/// @param req
/// @return
static esp_err_t monitor_get_handler(httpd_req_t *req)
{
    // const uint32_t L = setup_end - setup_start;

    ESP_LOGI(TAG, "Serve monitor");
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_send(req, monitor_start, monitor_len);
    auto_disable(); // stop automated control

    return ESP_OK;
}

static const httpd_uri_t monitor = {
    .uri = "/monitor",
    .method = HTTP_GET,
    .handler = monitor_get_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/get_acc"
static esp_err_t get_acc_handler(httpd_req_t *req)
{
    // 1. データの取得
    const char *p = get_unread_data();

    // 2. CORS ヘッダーの追加 (これが重要！)
    // これがないと、ブラウザがセキュリティエラーとして通信を遮断します
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // 3. レスポンス設定
    httpd_resp_set_type(req, "text/csv; charset=UTF-8");

    // 4. 送信
    httpd_resp_send(req, p, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static const httpd_uri_t get_acc = {
    .uri = "/get_acc",
    .method = HTTP_GET,
    .handler = get_acc_handler,
    .user_ctx = NULL};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/ota"
/// @param req
/// @return
static esp_err_t ota_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serve ota");
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_send(req, ota_start, ota_len);

    return ESP_OK;
}

static const httpd_uri_t ota = {
    .uri = "/ota",
    .method = HTTP_GET,
    .handler = ota_get_handler};

////////////////////////////////////////////////////////////////////////////
#define OTA_BUFF_SIZE 1024 // 受信バッファサイズ（1KB）

/// @brief  "POST /update" ハンドラ
///         PCから送られてくる生バイナリを逐次OTAパーティションに書き込みます。
static char rx_buffer[OTA_BUFF_SIZE];
static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "OTAアップデートを開始します...");

    // 1. 次のOTA書き込み対象パーティションを取得
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL)
    {
        ESP_LOGE(TAG, "OTAパーティションが見つかりません。");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "書き込み先パーティション: %s", update_partition->label);

    // 2. OTAの開始を宣言
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin 失敗: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int received;
    bool is_first_packet = true; // ★初回パケット判定用フラグ

    ESP_LOGI(TAG, "データを受信中... 全長: %d バイト", remaining);

    // 3. ループでPOSTデータを逐次受信し、フラッシュに書き込む
    while (remaining > 0)
    {
        int recv_to_read = (remaining < OTA_BUFF_SIZE) ? remaining : OTA_BUFF_SIZE;
        received = httpd_req_recv(req, rx_buffer, recv_to_read);

        if (received <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGW(TAG, "Socket timeout... 再試行します");
                continue;
            }
            ESP_LOGE(TAG, "データ受信中にエラーが発生しました。");
            esp_ota_abort(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        if (is_first_packet)
        {
            is_first_packet = false;

            // 先頭1バイトがESP32バイナリの識別子 0xE9 かどうかを確認
            if ((uint8_t)rx_buffer[0] != ESP32BINMARK)
            {
                ESP_LOGE(TAG, "不正なファイルです: マジックバイトが 0xE9 ではありません (0x%02X)", (uint8_t)rx_buffer[0]);
                esp_ota_abort(update_handle);
                // PC側に400 Bad Requestのエラーを返す
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware image (Missing 0xE9)");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "ファームウェアのヘッダー検証(0xE9)に成功しました。");
        }

        // データを逐次OTAパーティションに書き込み
        err = esp_ota_write(update_handle, (const void *)rx_buffer, received);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write 失敗: %s", esp_err_to_name(err));
            esp_ota_abort(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        remaining -= received;
    }

    ESP_LOGI(TAG, "全データの受信・書き込みが完了しました。");

    // 4. OTA書き込みの終了処理（チェックサムの検証）
    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end 失敗 (イメージの検証に失敗): %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 5. 次回起動するパーティションを切り替える
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition 失敗: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTAアップデートが成功しました。再起動します。");

    // クライアントへ成功レスポンス（ステータス200）を返す
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Success", HTTPD_RESP_USE_STRLEN);

    // レスポンスが確実に送信されるのを待ってから再起動
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

static const httpd_uri_t ota_update = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = ota_post_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/favicon.ico"
/// @param req
/// @return
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serve setup");
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, favicon_start, favicon_len);

    return ESP_OK;
}

static const httpd_uri_t favicon = {
    .uri = "/favicon.ico",
    .method = HTTP_ANY,
    .handler = favicon_get_handler};

/////////////////////////////////////////////////////////////////////////////
// captive portal response
// for android ///////////////////////////////////////
static esp_err_t generate_204_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/"); // ← 操作UI
    return httpd_resp_send(req, NULL, 0);
}
static const httpd_uri_t generate_204 = {
    .uri = "/generate_204",
    .method = HTTP_ANY,
    .handler = generate_204_handler};

// for iOS ///////////////////////////////////////////
static esp_err_t hotspot_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}
static const httpd_uri_t hotspot = {
    .uri = "/hotspot-detect.html",
    .method = HTTP_ANY,
    .handler = hotspot_handler};

// for Windows ///////////////////////////////////////
static esp_err_t ncsi_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}
static const httpd_uri_t ncsi = {
    .uri = "/ncsi.txt",
    .method = HTTP_ANY,
    .handler = ncsi_handler};

/////////////////////////////////////////////////////////////////////////////
// HTTP Error (404) Handler - Redirects all requests to the root page
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

/////////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

/////////////////////////////////////////////////////////////////////////////
static void wifi_init_softap(void)
{
    uint8_t mac[6];
    char ssid[32];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    esp_wifi_get_mac(WIFI_IF_AP, mac);
    snprintf(ssid, sizeof(ssid), "ROBOBIKE-%02X%02X%02X", mac[3], mac[4], mac[5]);

    int ssid_len = strlen(ssid);
    uint8_t channel = ((((uint16_t)mac[4] << 8 | (uint16_t)mac[5])) % 13) + 1; // 1 - 13
    wifi_config_t wifi_config = {
        .ap = {
            .password = "",
            .ssid_len = (uint8_t)strlen(ssid),
            .channel = channel, // 1〜13 のいずれかが入る
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    memcpy(wifi_config.ap.ssid, ssid, ssid_len + 1);

    if (strlen(ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
}

static void dhcp_set_captiveportal_url(void)
{
    // get the IP of the access point to redirect to
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    // turn the IP into a URI
    char *captiveportal_uri = (char *)malloc(32 * sizeof(char));
    assert(captiveportal_uri && "Failed to allocate captiveportal_uri");
    strcpy(captiveportal_uri, "http://");
    strcat(captiveportal_uri, ip_addr);

    // get a handle to configure DHCP with
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // set the DHCP option 114
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, captiveportal_uri, strlen(captiveportal_uri)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(netif));
}

/////////////////////////////////////////////////////////////////////////////
void webserver_start()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_netif_create_default_wifi_ap();
    wifi_init_softap();
    dhcp_set_captiveportal_url();

    httpd_config_t http_conf = HTTPD_DEFAULT_CONFIG();
    http_conf.server_port = 80;
    http_conf.ctrl_port = 32769;
    http_conf.max_open_sockets = 4;
    http_conf.lru_purge_enable = true;
    http_conf.stack_size = 6144;
    http_conf.max_uri_handlers = 20;

    ESP_LOGI(TAG, "Starting HTTP server for general pages on port: 80");
    if (httpd_start(&http_server, &http_conf) == ESP_OK)
    {
        httpd_register_uri_handler(http_server, &root);
        httpd_register_uri_handler(http_server, &command);
        httpd_register_uri_handler(http_server, &setup);
        httpd_register_uri_handler(http_server, &setup2);
        httpd_register_uri_handler(http_server, &monitor);
        httpd_register_uri_handler(http_server, &get_acc);
        httpd_register_uri_handler(http_server, &ota);
        httpd_register_uri_handler(http_server, &ota_update);
        httpd_register_uri_handler(http_server, &favicon);
        httpd_register_uri_handler(http_server, &generate_204);
        httpd_register_uri_handler(http_server, &hotspot);
        httpd_register_uri_handler(http_server, &ncsi);
        httpd_register_err_handler(http_server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }

    dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    start_dns_server(&config);

    wifi_config_t conf;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &conf));

    printf("Password=%s\n", conf.ap.password);
    printf("Channel=%d\n", conf.ap.channel);
    printf("Max_Connections=%d\n", conf.ap.max_connection);
    printf("Auth_Mode=%d\n", conf.ap.authmode);
    printf("AP_SSID=%s\n", conf.ap.ssid);
}
