#include "userdefine.h"
#include <esp_ota_ops.h>

static const char *TAG = "web_ota";

extern const char ota_start[] asm("_binary_ota_html_start");
extern const char ota_end[] asm("_binary_ota_html_end");
extern const int32_t ota_len asm("ota_html_length");


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

const httpd_uri_t ota = {
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

const httpd_uri_t ota_update = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = ota_post_handler};
