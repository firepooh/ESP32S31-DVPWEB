#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 80번 포트에서 HTTP 서버를 시작한다.
 *   /         웹 페이지
 *   /stream   MJPEG 스트림 (multipart/x-mixed-replace)
 *   /capture  JPEG 한 장
 *   /status   상태 JSON
 *   /control  센서 설정 (?var=<이름>&val=<정수>)
 */
esp_err_t web_server_start(void);

/** 현재 스트림 접속 수 */
int web_server_stream_clients(void);

#ifdef __cplusplus
}
#endif
