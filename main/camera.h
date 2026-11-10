#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool running;            /*!< 캡처 동작 여부 */
    const char *sensor;      /*!< 센서 이름 (예: OV3660) */
    const char *format;      /*!< 센서 포맷 이름 */
    const char *jpeg_source; /*!< "sensor": 센서 JPEG, "hw": S31 하드웨어 인코더 */
    uint16_t width;
    uint16_t height;
    float fps;               /*!< 최근 1초 동안의 프레임 속도 */
    uint32_t frames;         /*!< 정상 수신한 프레임 수 */
    uint32_t dropped;        /*!< 빈 버퍼가 없어 버린 프레임 수 */
    uint32_t bad;            /*!< JPEG 시작/끝 표시가 없어 버린 프레임 수 */
    size_t jpeg_size;        /*!< 마지막 JPEG 크기(byte) */
} cam_status_t;

/**
 * DVP 컨트롤러와 센서를 초기화하고 캡처를 시작한다.
 * OV3660은 센서 JPEG을 그대로 쓰고, SC101IOT는 YUV422를 하드웨어 JPEG 인코더로 압축한다.
 */
esp_err_t cam_init(void);

/**
 * *seq 보다 새로운 프레임을 호출자 버퍼에 복사한다.
 * 버퍼가 작으면 PSRAM에서 다시 할당하고 *buf, *cap을 갱신한다.
 * *seq = 0 이면 현재 프레임을 바로 돌려준다.
 */
esp_err_t cam_get_jpeg(uint32_t *seq, uint8_t **buf, size_t *cap, size_t *len, uint32_t timeout_ms);

void cam_get_status(cam_status_t *st);

/** 센서 파라미터를 설정한다. name: quality, ae_level, contrast, saturation, special_effect, wb_mode, hmirror, vflip */
esp_err_t cam_set_param(const char *name, int value);

/** 지원하는 파라미터와 현재 값을 JSON 객체로 쓴다. 쓴 길이를 반환한다. */
int cam_params_to_json(char *out, size_t size);

#ifdef __cplusplus
}
#endif
