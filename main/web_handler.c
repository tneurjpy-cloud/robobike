#include "userdefine.h"

static const char *TAG = "web_handler";

#define IF_BTEQ(str) if (strcmp(bt_name, str) == 0)
#define ELSE_IF_BTEQ(str) else if (strcmp(bt_name, str) == 0)

// static file server
typedef struct
{
    const char *start;
    const char *end;
    const char *type;
    bool disable_auto;
} file_server_data_t;

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/file"
static esp_err_t common_file_get_handler(httpd_req_t *req)
{
    file_server_data_t *data = (file_server_data_t *)req->user_ctx;
    if (strcmp(req->uri, "/setup") == 0)
        saved.isChecked = true;
    if (data->disable_auto)
        auto_disable();

    httpd_resp_set_type(req, data->type);
    return httpd_resp_send(req, data->start, data->end - data->start);
}

extern const char setup_start[] asm("_binary_setup_html_start");
extern const char setup_end[] asm("_binary_setup_html_end");
static const file_server_data_t d_setup = {
    setup_start, setup_end, "text/html; charset=UTF-8", true};
const httpd_uri_t setup = {
    .uri = "/setup",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_setup};

extern const char setup2_start[] asm("_binary_setup2_html_start");
extern const char setup2_end[] asm("_binary_setup2_html_end");
static const file_server_data_t d_setup2 = {
    setup2_start, setup2_end, "text/html; charset=UTF-8", true};
const httpd_uri_t setup2 = {
    .uri = "/setup2",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_setup2};

extern const char monitor_start[] asm("_binary_monitor_html_start");
extern const char monitor_end[] asm("_binary_monitor_html_end");
static const file_server_data_t d_monitor = {
    monitor_start, monitor_end, "text/html; charset=UTF-8", false};
const httpd_uri_t monitor = {
    .uri = "/monitor",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_monitor};

extern const char favicon_start[] asm("_binary_favicon_ico_start");
extern const char favicon_end[] asm("_binary_favicon_ico_end");
static const file_server_data_t d_favicon = {
    favicon_start, favicon_end, "image/x-icon", false};
const httpd_uri_t favicon = {
    .uri = "/favicon.ico",
    .method = HTTP_ANY,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_favicon};

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
            set_str_cmd(-STR_STOP, 10.0f);                                      // 左傾を誘発
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

const httpd_uri_t command = {
    .uri = "/command",
    .method = HTTP_GET,
    .handler = command_handler};

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/get_acc"
static esp_err_t get_acc_handler(httpd_req_t *req)
{
    // 1. データの取得
    const char *p = get_data();

    // 2. CORS ヘッダーの追加 (これが重要！)
    // これがないと、ブラウザがセキュリティエラーとして通信を遮断します
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/csv; charset=UTF-8");
    httpd_resp_send(req, p, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

const httpd_uri_t get_acc = {
    .uri = "/get_acc",
    .method = HTTP_GET,
    .handler = get_acc_handler,
    .user_ctx = NULL};

/////////////////////////////////////////////////////////////////////////////
// captive portal response
// for android ///////////////////////////////////////
static esp_err_t generate_204_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/"); // ← 操作UI
    return httpd_resp_send(req, NULL, 0);
}

const httpd_uri_t generate_204 = {
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

const httpd_uri_t hotspot = {
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

const httpd_uri_t ncsi = {
    .uri = "/ncsi.txt",
    .method = HTTP_ANY,
    .handler = ncsi_handler};

/////////////////////////////////////////////////////////////////////////////
// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

/////////////////////////////////////////////////////////////////////////////
// HTTP GET Handler
/// @brief  "http://192.168.4.1/"
static struct in_addr master_ip = {.s_addr = 0};
extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");
extern const int32_t root_len asm("root_html_length");
esp_err_t root_get_handler(httpd_req_t *req)
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
        return httpd_resp_send(req, monitor_start, monitor_end - monitor_start);
    }
}

const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler};
