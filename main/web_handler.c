#include "userdefine.h"

static const char *TAG = "web_handler";

// static file server
typedef struct
{
    const char *start;
    const char *end;
    const char *type;
    bool disable_auto;
} file_server_data_t;

/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/FILENAME"
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

// setup ////////////////////////////////////////////////////////////////
extern const char setup_start[] asm("_binary_setup_html_start");
extern const char setup_end[] asm("_binary_setup_html_end");
static const file_server_data_t d_setup = {
    setup_start, setup_end, "text/html; charset=UTF-8", true};
static const httpd_uri_t setup = {
    .uri = "/setup",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_setup};

// setup2 ////////////////////////////////////////////////////////////////
extern const char setup2_start[] asm("_binary_setup2_html_start");
extern const char setup2_end[] asm("_binary_setup2_html_end");
static const file_server_data_t d_setup2 = {
    setup2_start, setup2_end, "text/html; charset=UTF-8", true};
static const httpd_uri_t setup2 = {
    .uri = "/setup2",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_setup2};

// monitor ////////////////////////////////////////////////////////////////
extern const char monitor_start[] asm("_binary_monitor_html_start");
extern const char monitor_end[] asm("_binary_monitor_html_end");
static const file_server_data_t d_monitor = {
    monitor_start, monitor_end, "text/html; charset=UTF-8", false};
static const httpd_uri_t monitor = {
    .uri = "/monitor",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_monitor};

// connecttest.txt for dummy ////////////////////////////////////////////////
extern const char connecttest_start[] asm("_binary_connecttest_txt_start");
extern const char connecttest_end[] asm("_binary_connecttest_txt_end");
static const file_server_data_t d_connecttest = {
    connecttest_start, connecttest_end, "text/html; charset=UTF-8", false};

static const httpd_uri_t connecttest = {
    .uri = "/connecttest.txt",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_connecttest};

static const httpd_uri_t hotspot_detect = {
    .uri = "/hotspot-detect.html",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_connecttest};

static const httpd_uri_t success = {
    .uri = "/library/test/success.html",
    .method = HTTP_GET,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_connecttest};

// favicon ////////////////////////////////////////////////////////////////
extern const char favicon_start[] asm("_binary_favicon_ico_start");
extern const char favicon_end[] asm("_binary_favicon_ico_end");
static const file_server_data_t d_favicon = {
    favicon_start, favicon_end, "image/x-icon", false};
static const httpd_uri_t favicon = {
    .uri = "/favicon.ico",
    .method = HTTP_ANY,
    .handler = common_file_get_handler,
    .user_ctx = (void *)&d_favicon};

////////////////////////////////////////////////////////////////////////////////////

/// TASK "httpd" ////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/// @brief  "http://192.168.4.1/command?button=13"
static esp_err_t command_handler(httpd_req_t *req)
{
    char query_buf[64];
    char val_str[32];

    userLastControlTime = millis();
    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (httpd_query_key_value(query_buf, "button", val_str, sizeof(val_str)) != ESP_OK)
    {
        return ESP_FAIL;
    }

    control_msg_t msg;
    msg.req = req;
    msg.id = (TcmdID)atoi(val_str);
    put_command(&msg);

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_send(req, get_edit_data(), HTTPD_RESP_USE_STRLEN);
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
    char *p = get_control_data();
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
/// @brief  "http://192.168.4.1/clear_buffer"
static esp_err_t clear_buffer_handler(httpd_req_t *req)
{
    char *p = clear_buffer_data();
    httpd_resp_set_type(req, "text/plane; charset=UTF-8");
    httpd_resp_send(req, p, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const httpd_uri_t clear_buffer = {
    .uri = "/clear_buffer",
    .method = HTTP_GET,
    .handler = clear_buffer_handler,
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
// "ROBOBIKE CONTROL"
// http://192.168.4.1/
// [MASTER]: 1st client [GUEST]: 2nd
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
                client_ip.s_addr = 0; // 純IPv6は無視
            }
        }
    }

    ESP_LOGI(TAG, "Request [FD:%d] [IP:%s] [Port:%d]", fd, ip_str, port);

    if (!saved.isChecked)
    {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/setup");
        return httpd_resp_send(req, NULL, 0);
    }

    if (client_ip.s_addr != 0 && master_ip.s_addr == 0)
    {
        master_ip = client_ip;

        char master_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &master_ip, master_str, sizeof(master_str));

        ESP_LOGI(TAG, "★★★ MASTER REGISTERED: %s ★★★", master_str);
    }

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

//////////////////////////////////////////////////////////////////////////
// uri handlers declaration //////////////////////////////////////////////
extern const httpd_uri_t ota_get, ota_update;
const httpd_uri_t *uri_handlers[] = {
    &root, &ncsi, &generate_204, &get_acc, &clear_buffer, &command, &ota_update, &ota_get,
    &setup, &setup2, &monitor, &connecttest, &hotspot_detect, &success,
    &favicon};
const size_t uri_handlers_count = sizeof(uri_handlers) / sizeof(uri_handlers[0]);

//////////////////////////////////////////////////////////////////////////
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
