#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_SSID_MAX_LEN   32
#define WIFI_PASS_MAX_LEN   64

/** netif와 Wi-Fi 드라이버를 STA 모드로 초기화한다. 접속은 하지 않는다. */
esp_err_t wifi_sta_init(void);

/** NVS에 저장된 SSID/비밀번호를 읽는다. 저장값이 없으면 false */
bool wifi_sta_load_creds(char *ssid, size_t ssid_size, char *pass, size_t pass_size);

esp_err_t wifi_sta_save_creds(const char *ssid, const char *pass);

esp_err_t wifi_sta_erase_creds(void);

/** 주어진 AP로 접속을 시작한다. 연결이 끊어지면 계속 재접속한다. */
esp_err_t wifi_sta_connect(const char *ssid, const char *pass);

/** IP를 받을 때까지 최대 timeout_ms 동안 기다린다. */
bool wifi_sta_wait_connected(uint32_t timeout_ms);

bool wifi_sta_is_connected(void);

/** 접속 대상 SSID. 아직 설정 전이면 빈 문자열 */
const char *wifi_sta_get_ssid(void);

/** 현재 IP 주소 문자열. 연결 전이면 "0.0.0.0" */
void wifi_sta_get_ip(char *buf, size_t len);

/** AP 신호 세기(dBm). 연결 전이면 0 */
int wifi_sta_get_rssi(void);

#ifdef __cplusplus
}
#endif
