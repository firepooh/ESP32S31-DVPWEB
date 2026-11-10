#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "wifi_sta.h"

#define CONNECTED_BIT   BIT0
#define NVS_NAMESPACE   "wifi_cfg"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASS    "pass"

static const char *TAG = "wifi_sta";

static EventGroupHandle_t s_evt;
static esp_netif_t *s_netif;
static esp_ip4_addr_t s_ip;
static int s_retry;
static char s_ssid[WIFI_SSID_MAX_LEN + 1];

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *ev = data;
        xEventGroupClearBits(s_evt, CONNECTED_BIT);
        s_ip.addr = 0;
        s_retry++;
        ESP_LOGW(TAG, "Disconnected from \"%s\" (reason %d), reconnecting... (#%d)", s_ssid, ev->reason, s_retry);
        if (s_retry % 5 == 0) {
            ESP_LOGW(TAG, "If the SSID or password is wrong, type 'wifi_reset' on this console to enter it again.");
        }
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        s_ip = ev->ip_info.ip;
        s_retry = 0;
        ESP_LOGI(TAG, "Got IP: " IPSTR "  ->  http://" IPSTR "/", IP2STR(&s_ip), IP2STR(&s_ip));
        xEventGroupSetBits(s_evt, CONNECTED_BIT);
    }
}

static void copy_str(uint8_t *dst, size_t dst_size, const char *src)
{
    size_t n = strnlen(src, dst_size);
    memset(dst, 0, dst_size);
    memcpy(dst, src, n);
}

esp_err_t wifi_sta_init(void)
{
    s_evt = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_evt, ESP_ERR_NO_MEM, TAG, "event group");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_netif, CONFIG_APP_WIFI_HOSTNAME);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "esp_wifi_init");
    // 접속 정보는 앱이 NVS에서 직접 관리하므로 드라이버 쪽 저장은 끈다.
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set_storage");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                            on_wifi_event, NULL, NULL), TAG, "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                            on_wifi_event, NULL, NULL), TAG, "ip handler");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set_mode");
    return ESP_OK;
}

bool wifi_sta_load_creds(char *ssid, size_t ssid_size, char *pass, size_t pass_size)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t l1 = ssid_size;
    size_t l2 = pass_size;
    bool ok = nvs_get_str(h, NVS_KEY_SSID, ssid, &l1) == ESP_OK &&
              nvs_get_str(h, NVS_KEY_PASS, pass, &l2) == ESP_OK &&
              ssid[0] != '\0';
    nvs_close(h);
    return ok;
}

esp_err_t wifi_sta_save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "nvs_open");
    esp_err_t err = nvs_set_str(h, NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t wifi_sta_erase_creds(void)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "nvs_open");
    esp_err_t err = nvs_erase_all(h);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t wifi_sta_connect(const char *ssid, const char *pass)
{
    strlcpy(s_ssid, ssid, sizeof(s_ssid));

    wifi_config_t wc = {0};
    copy_str(wc.sta.ssid, sizeof(wc.sta.ssid), ssid);
    copy_str(wc.sta.password, sizeof(wc.sta.password), pass);
    // 비밀번호가 비어 있으면 개방형 AP로 접속한다.
    wc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wc.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wc), TAG, "set_config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
    // 스트리밍 지연과 프레임 끊김을 줄이려고 Wi-Fi 절전을 끈다.
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Connecting to SSID \"%s\"...", s_ssid);
    return ESP_OK;
}

bool wifi_sta_wait_connected(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(s_evt, CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (bits & CONNECTED_BIT) != 0;
}

bool wifi_sta_is_connected(void)
{
    return s_evt && (xEventGroupGetBits(s_evt) & CONNECTED_BIT);
}

const char *wifi_sta_get_ssid(void)
{
    return s_ssid;
}

void wifi_sta_get_ip(char *buf, size_t len)
{
    snprintf(buf, len, IPSTR, IP2STR(&s_ip));
}

int wifi_sta_get_rssi(void)
{
    wifi_ap_record_t ap;
    if (wifi_sta_is_connected() && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}
