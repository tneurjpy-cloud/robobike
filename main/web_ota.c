#include "userdefine.h"
#include <esp_ota_ops.h>

#define ESP32BINMARK 0xE9

static const char *TAG = "web_ota";

extern const char ota_start[] asm("_binary_ota_html_start");
extern const char ota_end[] asm("_binary_ota_html_end");
extern const int32_t ota_len asm("ota_html_length");

/////////////////////////////////////////////////////////////////////////////
/// @brief  Endpoint: "http://192.168.4.1/ota"
/// @param req
/// @return
static esp_err_t ota_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serve ota page");
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_send(req, ota_start, ota_len);

    return ESP_OK;
}

const httpd_uri_t ota_get = {
    .uri = "/ota",
    .method = HTTP_GET,
    .handler = ota_get_handler};

////////////////////////////////////////////////////////////////////////////
#define OTA_BUFF_SIZE 1024 // Receive buffer size (1KB)

/// @brief  "POST /update" handler
///         Sequentially writes raw binary sent from PC to the OTA partition.
static char rx_buffer[OTA_BUFF_SIZE];
static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA update...");

    // 1. Get the next OTA partition to write
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL)
    {
        ESP_LOGE(TAG, "OTA partition not found.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition: %s", update_partition->label);

    // 2. Begin OTA update
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int received;
    bool is_first_packet = true; // Flag for first packet validation

    ESP_LOGI(TAG, "Receiving data... Total length: %d bytes", remaining);

    // 3. Loop to receive POST data and write to flash
    while (remaining > 0)
    {
        int recv_to_read = (remaining < OTA_BUFF_SIZE) ? remaining : OTA_BUFF_SIZE;
        received = httpd_req_recv(req, rx_buffer, recv_to_read);

        if (received <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGW(TAG, "Socket timeout... Retrying");
                continue;
            }
            ESP_LOGE(TAG, "Error occurred during data reception.");
            esp_ota_abort(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        if (is_first_packet)
        {
            is_first_packet = false;

            // Check if the first byte is the ESP32 binary identifier (0xE9)
            if ((uint8_t)rx_buffer[0] != ESP32BINMARK)
            {
                ESP_LOGE(TAG, "Invalid file: Magic byte is not 0xE9 (0x%02X)", (uint8_t)rx_buffer[0]);
                esp_ota_abort(update_handle);
                // Return 400 Bad Request to the client
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware image (Missing 0xE9)");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Firmware header validation (0xE9) successful.");
        }

        // Write data to OTA partition sequentially
        err = esp_ota_write(update_handle, (const void *)rx_buffer, received);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        remaining -= received;
    }

    ESP_LOGI(TAG, "All data received and written successfully.");

    // 4. Finalize OTA writing (checksum validation)
    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed (Image validation failed): %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 5. Switch to the new boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA update successful. Rebooting...");

    // Send success response (Status 200) to client
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Success", HTTPD_RESP_USE_STRLEN);

    // Wait a moment for the response to be sent, then restart
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

const httpd_uri_t ota_update = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = ota_post_handler};