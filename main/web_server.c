#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "camera.h"
#include "wifi_sta.h"
#include "web_server.h"

#define PART_BOUNDARY "s31dvpcamframe"

static const char *TAG = "web";

static const char STREAM_CONTENT_TYPE[] = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char STREAM_PART_FMT[] = "\r\n--" PART_BOUNDARY "\r\n"
                                      "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t s_server;
static atomic_int s_stream_clients;

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    uint32_t seq = 0;
    uint8_t *buf = NULL;
    size_t cap = 0;
    size_t len = 0;

    esp_err_t err = cam_get_jpeg(&seq, &buf, &cap, &len, 2000);
    if (err != ESP_OK) {
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera frame not available");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    err = httpd_resp_send(req, (const char *)buf, len);
    free(buf);
    return err;
}

/* 스트림 접속마다 태스크 하나가 프레임을 보낸다. 서버 태스크는 막히지 않는다. */
static void stream_task(void *arg)
{
    httpd_req_t *req = arg;
    int fd = httpd_req_to_sockfd(req);
    uint32_t seq = 0;
    uint8_t *buf = NULL;
    size_t cap = 0;
    size_t len = 0;
    uint32_t sent = 0;
    int64_t t0 = esp_timer_get_time();
    char part[128];

    ESP_LOGI(TAG, "Stream client connected (fd=%d, clients=%d)", fd, atomic_load(&s_stream_clients));

    // 작은 조각이 지연 ACK를 기다리며 멈추지 않도록 Nagle 알고리즘을 끈다.
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");

    while (true) {
        if (cam_get_jpeg(&seq, &buf, &cap, &len, 3000) != ESP_OK) {
            ESP_LOGW(TAG, "No camera frame, closing stream (fd=%d)", fd);
            break;
        }
        int hlen = snprintf(part, sizeof(part), STREAM_PART_FMT, (unsigned)len);
        if (httpd_resp_send_chunk(req, part, hlen) != ESP_OK ||
                httpd_resp_send_chunk(req, (const char *)buf, len) != ESP_OK) {
            break;  // 브라우저가 연결을 끊었다.
        }
        sent++;
    }

    float secs = (float)(esp_timer_get_time() - t0) / 1e6f;
    ESP_LOGI(TAG, "Stream client closed (fd=%d, %" PRIu32 " frames, %.1f fps)",
             fd, sent, secs > 0 ? (float)sent / secs : 0.0f);

    free(buf);
    httpd_req_async_handler_complete(req);
    httpd_sess_trigger_close(s_server, fd);
    atomic_fetch_sub(&s_stream_clients, 1);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (atomic_fetch_add(&s_stream_clients, 1) >= CONFIG_APP_STREAM_MAX_CLIENTS) {
        atomic_fetch_sub(&s_stream_clients, 1);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "3");
        return httpd_resp_sendstr(req, "Too many stream clients");
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        atomic_fetch_sub(&s_stream_clients, 1);
        return ESP_FAIL;
    }
    if (xTaskCreatePinnedToCore(stream_task, "mjpeg_stream", 5120, async_req, 5, NULL, tskNO_AFFINITY) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        atomic_fetch_sub(&s_stream_clients, 1);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    cam_status_t st;
    cam_get_status(&st);

    char ip[16];
    wifi_sta_get_ip(ip, sizeof(ip));

    char params[640];
    cam_params_to_json(params, sizeof(params));

    const size_t size = 1400;
    char *json = malloc(size);
    if (json == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    }

    int n = snprintf(json, size,
                     "{\"camera\":{\"running\":%s,\"sensor\":\"%s\",\"format\":\"%s\",\"jpeg_source\":\"%s\","
                     "\"width\":%u,\"height\":%u,"
                     "\"fps\":%.1f,\"frames\":%" PRIu32 ",\"dropped\":%" PRIu32 ",\"bad\":%" PRIu32 ",\"jpeg_size\":%u},"
                     "\"params\":%s,"
                     "\"wifi\":{\"ip\":\"%s\",\"rssi\":%d},"
                     "\"stream_clients\":%d,\"max_stream_clients\":%d,"
                     "\"heap\":{\"internal\":%u,\"psram\":%u},"
                     "\"uptime_s\":%lld}",
                     st.running ? "true" : "false", st.sensor, st.format, st.jpeg_source, st.width, st.height,
                     st.fps, st.frames, st.dropped, st.bad, (unsigned)st.jpeg_size,
                     params,
                     ip, wifi_sta_get_rssi(),
                     atomic_load(&s_stream_clients), CONFIG_APP_STREAM_MAX_CLIENTS,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                     (long long)(esp_timer_get_time() / 1000000));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, json, n < (int)size ? n : (int)size - 1);
    free(json);
    return err;
}

static esp_err_t control_handler(httpd_req_t *req)
{
    char query[96];
    char var[32];
    char val[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
            httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
            httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Usage: /control?var=<name>&val=<int>");
    }

    char *end = NULL;
    long v = strtol(val, &end, 10);
    if (end == val || *end != '\0') {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "val must be an integer");
    }

    esp_err_t err = cam_set_param(var, (int)v);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, esp_err_to_name(err));
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t web_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 6144;
    cfg.max_open_sockets = CONFIG_APP_STREAM_MAX_CLIENTS + 4;
    cfg.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",            .method = HTTP_GET, .handler = index_handler },
        { .uri = "/stream",      .method = HTTP_GET, .handler = stream_handler },
        { .uri = "/capture",     .method = HTTP_GET, .handler = capture_handler },
        { .uri = "/status",      .method = HTTP_GET, .handler = status_handler },
        { .uri = "/control",     .method = HTTP_GET, .handler = control_handler },
        { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", cfg.server_port);
    return ESP_OK;
}

int web_server_stream_clients(void)
{
    return atomic_load(&s_stream_clients);
}
