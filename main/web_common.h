#include <esp_http_server.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

extern const httpd_uri_t root;
extern const httpd_uri_t command;
extern const httpd_uri_t setup;
extern const httpd_uri_t setup2;
extern const httpd_uri_t monitor;
extern const httpd_uri_t get_acc;
extern const httpd_uri_t favicon;
extern const httpd_uri_t generate_204;
extern const httpd_uri_t hotspot;
extern const httpd_uri_t ncsi;
extern const httpd_uri_t generate_204;

extern const httpd_uri_t ota_update;
extern const httpd_uri_t ota;

char *mkcsv();
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);
