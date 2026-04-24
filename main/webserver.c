/*
CmakeList.txt は下記とする
idf_component_register(SRCS main.c servo.c IMU.c icm426xx.c userdevice.c webserver.c
   EMBED_FILES root.html setup.html setup2.html ota.html)
*/
#include "userdefine.h"

static const char *TAG = "websrv";

static httpd_handle_t http_server = NULL;


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
