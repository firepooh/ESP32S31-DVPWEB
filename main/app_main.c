/*
 * ESP32-S31-Korvo-1 DVP 카메라(OV3660) 웹 스트리밍
 *
 * Wi-Fi STA로 공유기에 접속한 뒤 http://<IP>/ 에서 영상을 보여 준다.
 * 처음 부팅하면 시리얼 콘솔(UART0, 보드의 USB-UART 포트)에서 SSID와 비밀번호를 입력받아 NVS에 저장한다.
 * 저장값이 있으면 다시 묻지 않는다. 바꾸려면 콘솔에서 wifi_reset을 입력한다.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "camera.h"
#include "console.h"
#include "web_server.h"
#include "wifi_sta.h"

static const char *TAG = "app";

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void prompt_wifi_creds(char *ssid, size_t ssid_size, char *pass, size_t pass_size)
{
    printf("\n"
           "============================================================\n"
           " Wi-Fi setup: credentials are saved to NVS and asked only once.\n"
           " (type 'wifi_reset' later to change them)\n"
           "============================================================\n");
    fflush(stdout);

    do {
        console_read_line("Wi-Fi SSID: ", ssid, ssid_size, false, 15000);
    } while (ssid[0] == '\0');

    while (true) {
        int n = console_read_line("Wi-Fi password (empty = open network): ", pass, pass_size, true, 0);
        if (n == 0 || (n >= 8 && n <= WIFI_PASS_MAX_LEN)) {
            break;
        }
        printf("WPA2/WPA3 password must be 8 to 63 characters (or 64 hex digits). Try again.\n");
        fflush(stdout);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S31-Korvo-1 DVP web camera");
    ESP_LOGI(TAG, "Free heap: internal %u, PSRAM %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    nvs_init();
    ESP_ERROR_CHECK(console_init());

    // 카메라가 실패해도 웹 서버는 띄워서 /status 로 원인을 볼 수 있게 한다.
    esp_err_t cam_err = cam_init();
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(cam_err));
    }

    ESP_ERROR_CHECK(wifi_sta_init());
    ESP_ERROR_CHECK(web_server_start());

    char ssid[WIFI_SSID_MAX_LEN + 1] = {0};
    char pass[WIFI_PASS_MAX_LEN + 1] = {0};
    if (wifi_sta_load_creds(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "Using saved Wi-Fi credentials (SSID \"%s\")", ssid);
    } else {
        prompt_wifi_creds(ssid, sizeof(ssid), pass, sizeof(pass));
        esp_err_t err = wifi_sta_save_creds(ssid, pass);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi credentials saved to NVS (SSID \"%s\")", ssid);
        } else {
            ESP_LOGE(TAG, "Failed to save Wi-Fi credentials: %s", esp_err_to_name(err));
        }
    }
    ESP_ERROR_CHECK(wifi_sta_connect(ssid, pass));
    memset(pass, 0, sizeof(pass));

    console_start_commands();

    char ip[16];
    if (wifi_sta_wait_connected(30000)) {
        wifi_sta_get_ip(ip, sizeof(ip));
        ESP_LOGI(TAG, "Open http://%s/ in a browser", ip);
    } else {
        ESP_LOGW(TAG, "Wi-Fi not connected yet. Type 'wifi_reset' on the console to re-enter SSID/password.");
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        cam_status_t st;
        cam_get_status(&st);
        wifi_sta_get_ip(ip, sizeof(ip));
        ESP_LOGI(TAG, "cam %.1f fps, frames %" PRIu32 ", dropped %" PRIu32 ", bad %" PRIu32 ", jpeg %u B | "
                 "clients %d | ip %s rssi %d | heap int %u psram %u",
                 st.fps, st.frames, st.dropped, st.bad, (unsigned)st.jpeg_size,
                 web_server_stream_clients(), ip, wifi_sta_get_rssi(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}
