#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/jpeg_encode.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_xclk.h"
#include "camera.h"

/*
 * ESP32-S31-Korvo-1 DVP 카메라 핀 (회로도 기준)
 * - 데이터선 Y2~Y9 → D0~D7 (GPIO46~53)
 * - CAM_RESET은 10 kΩ 풀업, CAM_PWDN은 10 kΩ 풀다운으로 고정되어 GPIO 제어가 없다.
 * - SCCB는 GPIO0/1 이며 오디오 코덱(ES8389)과 같은 버스를 쓴다.
 *
 * 보드에 따라 OV3660(3 MP, 주소 0x3C) 또는 SC101IOT(1 MP, 주소 0x68)가 실려 있다.
 * - OV3660  : 센서가 JPEG을 직접 출력한다.
 * - SC101IOT: YUV422(UYVY)를 받아 S31 하드웨어 JPEG 인코더로 압축한다.
 */
#define CAM_PIN_D0          46
#define CAM_PIN_D1          47
#define CAM_PIN_D2          48
#define CAM_PIN_D3          49
#define CAM_PIN_D4          50
#define CAM_PIN_D5          51
#define CAM_PIN_D6          52
#define CAM_PIN_D7          53
#define CAM_PIN_PCLK        54
#define CAM_PIN_XCLK        55
#define CAM_PIN_VSYNC       56
#define CAM_PIN_HREF        57
#define CAM_PIN_SCCB_SDA    0
#define CAM_PIN_SCCB_SCL    1
#define CAM_I2C_PORT        I2C_NUM_0

#define CAM_DETECT_XCLK_HZ  20000000    // 센서 감지 때 쓰는 XCLK (Korvo-1 BSP와 같다)
#define CAM_WIDTH           CONFIG_APP_CAM_WIDTH    // 부팅할 때 쓰는 해상도
#define CAM_HEIGHT          CONFIG_APP_CAM_HEIGHT
#define CAM_FB_COUNT        CONFIG_APP_CAM_FB_COUNT
#define CAM_SWITCH_TIMEOUT_MS 5000      // 해상도 전환을 기다리는 최대 시간

typedef struct {
    uint8_t *buf;
    size_t len;
} frame_t;

typedef struct {
    const char *name;
    uint32_t id;
    bool supported;
    int min;
    int max;
    int value;
} cam_param_t;

static const char *TAG = "camera";

static cam_param_t s_params[] = {
    { .name = "quality",        .id = ESP_CAM_SENSOR_JPEG_QUALITY },
    { .name = "ae_level",       .id = ESP_CAM_SENSOR_AE_LEVEL },
    { .name = "contrast",       .id = ESP_CAM_SENSOR_CONTRAST },
    { .name = "saturation",     .id = ESP_CAM_SENSOR_SATURATION },
    { .name = "special_effect", .id = ESP_CAM_SENSOR_SPECIAL_EFFECT },
    { .name = "wb_mode",        .id = ESP_CAM_SENSOR_WB },
    { .name = "hmirror",        .id = ESP_CAM_SENSOR_HMIRROR },
    { .name = "vflip",          .id = ESP_CAM_SENSOR_VFLIP },
};
#define PARAM_COUNT (sizeof(s_params) / sizeof(s_params[0]))

static esp_cam_ctlr_handle_t s_ctlr;
static esp_cam_sensor_device_t *s_sensor;
static esp_cam_sensor_xclk_handle_t s_xclk;
static int s_xclk_hz;
static i2c_master_bus_handle_t s_i2c_bus;
static esp_cam_sensor_format_array_t s_fmts;    // 센서가 지원하는 포맷 (드라이버의 정적 배열)
static const esp_cam_sensor_format_t *s_cur;    // 사용 중인 포맷 (s_fmts 안의 항목)
static esp_cam_sensor_format_t s_fmt;           // s_cur의 사본 (상태 조회용)
static bool s_sensor_jpeg;                      // true: 센서 JPEG, false: YUV422 + 하드웨어 인코더

// 해상도에 따라 크기가 바뀌는 버퍼. 해상도를 바꿀 때 모두 놓고 다시 잡는다.
static size_t s_fb_size;
static uint8_t *s_fbs[CAM_FB_COUNT];
static uint8_t *s_drop_buf;     // 빈 버퍼가 없을 때 DMA가 쓰는 버림용 버퍼
static QueueHandle_t s_free_q;  // DMA에 넘길 수 있는 버퍼 (uint8_t *)
static QueueHandle_t s_ready_q; // 수신이 끝난 프레임 (frame_t)

// 하드웨어 JPEG 인코더 (YUV422 센서용)
static SemaphoreHandle_t s_enc_mutex;   // 재시작 직전 인코더가 쉬고 있는지 확인하는 데 쓴다.
static jpeg_encoder_handle_t s_enc;
static jpeg_encode_cfg_t s_enc_cfg;
static uint8_t *s_jpg_bufs[2];  // 압축 결과 버퍼 두 개: 하나는 최신 프레임, 하나는 다음 인코딩용
static uint8_t *s_jpg_spare;    // 다음 인코딩 결과를 쓸 버퍼
static size_t s_jpg_cap;

// 센서 SCCB 접근과 캡처 구성 변경을 직렬화한다 (재귀 잠금).
static SemaphoreHandle_t s_cam_lock;

// 해상도 전환 요청. 실제 전환은 cam_task가 한다.
static TaskHandle_t s_task;
static SemaphoreHandle_t s_req_lock;    // 요청하는 쪽끼리 직렬화
static SemaphoreHandle_t s_req_done;    // cam_task가 전환을 마치면 준다.
static const esp_cam_sensor_format_t *volatile s_req_fmt;
static esp_err_t s_req_err;

// 가장 최근 JPEG 프레임. 읽는 쪽은 잠금 안에서 복사만 한다.
static pthread_mutex_t s_lock;
static pthread_cond_t s_cond;
static frame_t s_latest;
static uint32_t s_seq;
static volatile bool s_running;

static volatile uint32_t s_frames;
static volatile uint32_t s_dropped;
static volatile uint32_t s_bad;
static float s_fps;

static void cam_lock(void)
{
    xSemaphoreTakeRecursive(s_cam_lock, portMAX_DELAY);
}

static void cam_unlock(void)
{
    xSemaphoreGiveRecursive(s_cam_lock);
}

/* ---------- DVP 인터럽트 콜백 ---------- */

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    BaseType_t woken = pdFALSE;
    uint8_t *buf = NULL;

    if (xQueueReceiveFromISR(s_free_q, &buf, &woken) != pdTRUE) {
        buf = s_drop_buf;
    }
    trans->buffer = buf;
    trans->buflen = s_fb_size;
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    BaseType_t woken = pdFALSE;
    uint8_t *buf = trans->buffer;

    if (buf == s_drop_buf) {
        s_dropped++;
        return false;
    }

    // JPEG 모드에서 received_size가 0이면 JPEG 시작(FFD8)이나 끝(FFD9)을 찾지 못한 프레임이다.
    if (trans->received_size == 0) {
        s_bad++;
        xQueueSendFromISR(s_free_q, &buf, &woken);
        return woken == pdTRUE;
    }

    frame_t f = { .buf = buf, .len = trans->received_size };
    if (xQueueSendFromISR(s_ready_q, &f, &woken) != pdTRUE) {
        s_dropped++;
        xQueueSendFromISR(s_free_q, &buf, &woken);
    }
    return woken == pdTRUE;
}

/* ---------- 센서 ---------- */

static cam_param_t *find_param(const char *name)
{
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(s_params[i].name, name) == 0) {
            return &s_params[i];
        }
    }
    return NULL;
}

/** 파라미터 범위와 기본값을 센서에서 읽고, 설정 메뉴의 기본값을 적용한다. */
static void params_init(void)
{
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        s_params[i].supported = false;
        esp_cam_sensor_param_desc_t desc = { .id = s_params[i].id };
        if (esp_cam_sensor_query_para_desc(s_sensor, &desc) == ESP_OK &&
                desc.type == ESP_CAM_SENSOR_PARAM_TYPE_NUMBER) {
            s_params[i].supported = true;
            s_params[i].min = desc.number.minimum;
            s_params[i].max = desc.number.maximum;
            s_params[i].value = desc.default_value;
        }
    }
    if (!s_sensor_jpeg) {
        // YUV 센서의 "quality"는 하드웨어 JPEG 인코더 품질(1~100)이다.
        cam_param_t *q = find_param("quality");
        q->supported = true;
        q->min = 1;
        q->max = 100;
        q->value = CONFIG_APP_HWJPEG_QUALITY;
    }
#if CONFIG_APP_CAM_ROTATE_180
    // 출고 예제와 같이 OV3660만 180도 돌린다 (SC101IOT는 바로 서 있다).
    if (strcmp(esp_cam_sensor_get_name(s_sensor), "OV3660") == 0) {
        cam_set_param("hmirror", 1);
        cam_set_param("vflip", 1);
    }
#endif
#if CONFIG_APP_CAM_JPEG_QUALITY > 0
    if (s_sensor_jpeg) {
        cam_set_param("quality", CONFIG_APP_CAM_JPEG_QUALITY);
    }
#endif
}

/** 포맷을 바꾸면 센서 레지스터가 초기화되므로, 사용자가 맞춘 값을 다시 쓴다. */
static void params_reapply(void)
{
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        cam_param_t *p = &s_params[i];
        if (!p->supported || (!s_sensor_jpeg && p->id == ESP_CAM_SENSOR_JPEG_QUALITY)) {
            continue;   // 하드웨어 인코더 품질은 인코더 설정 때 반영한다.
        }
        esp_err_t err = esp_cam_sensor_set_para_value(s_sensor, p->id, &p->value, sizeof(p->value));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Reapply %s = %d failed: %s", p->name, p->value, esp_err_to_name(err));
        }
    }
}

static esp_err_t xclk_start(int freq_hz)
{
    esp_cam_sensor_xclk_config_t cfg = {
        .ledc_cfg = {
            .timer = LEDC_TIMER_0,
            .clk_cfg = LEDC_AUTO_CLK,
            .channel = LEDC_CHANNEL_0,
            .xclk_freq_hz = freq_hz,
            .xclk_pin = CAM_PIN_XCLK,
        },
    };
    if (s_xclk == NULL) {
        ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_LEDC, &s_xclk), TAG, "xclk allocate");
    } else {
        esp_cam_sensor_xclk_stop(s_xclk);
    }
    ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_start(s_xclk, &cfg), TAG, "xclk start");
    s_xclk_hz = freq_hz;
    return ESP_OK;
}

static esp_err_t sensor_detect(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CAM_I2C_PORT,
        .sda_io_num = CAM_PIN_SCCB_SDA,
        .scl_io_num = CAM_PIN_SCCB_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "i2c_new_master_bus");

    esp_cam_sensor_config_t cfg = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = CAM_PIN_XCLK,
        .xclk_freq_hz = CAM_DETECT_XCLK_HZ,
        .sensor_port = ESP_CAM_SENSOR_DVP,
    };

    esp_cam_sensor_detect_fn_t *start = NULL;
    esp_cam_sensor_detect_fn_t *end = NULL;
    esp_cam_sensor_detect_get_array(&start, &end);

    for (esp_cam_sensor_detect_fn_t *p = start; p < end; ++p) {
        if (p->port != ESP_CAM_SENSOR_DVP) {
            continue;
        }
        sccb_i2c_config_t i2c_cfg = {
            .scl_speed_hz = CONFIG_APP_CAM_SCCB_FREQ_HZ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        ESP_RETURN_ON_ERROR(sccb_new_i2c_io(s_i2c_bus, &i2c_cfg, &cfg.sccb_handle), TAG, "sccb_new_i2c_io");
        s_sensor = p->detect(&cfg);
        if (s_sensor) {
            break;
        }
        esp_sccb_del_i2c_io(cfg.sccb_handle);
    }

    if (s_sensor == NULL) {
        // 원인 파악용: 버스에서 응답하는 장치를 모두 찾아 출력한다.
        ESP_LOGE(TAG, "No camera sensor detected on SCCB (SDA=%d, SCL=%d). Scanning I2C bus...",
                 CAM_PIN_SCCB_SDA, CAM_PIN_SCCB_SCL);
        int found = 0;
        for (uint16_t addr = 0x08; addr < 0x78; addr++) {
            if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
                ESP_LOGW(TAG, "  I2C device found at 0x%02x", addr);
                found++;
            }
        }
        ESP_LOGW(TAG, "  %d device(s) found (OV3660 = 0x3c, SC101IOT = 0x68)", found);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Detected sensor: %s", esp_cam_sensor_get_name(s_sensor));
    return ESP_OK;
}

/**
 * 해상도에 맞는 포맷을 고른다. 센서 JPEG을 먼저 찾고, 없으면 YUV422(UYVY) 중 가장 빠른 것을 쓴다.
 * 쓸 수 있는 포맷이 없으면 NULL을 돌려준다.
 */
static const esp_cam_sensor_format_t *pick_format(uint16_t width, uint16_t height)
{
    const esp_cam_sensor_format_t *jpeg = NULL;
    const esp_cam_sensor_format_t *yuv = NULL;
    for (uint32_t i = 0; i < s_fmts.count; i++) {
        const esp_cam_sensor_format_t *f = &s_fmts.format_array[i];
        if (f->width != width || f->height != height) {
            continue;
        }
        if (f->format == ESP_CAM_SENSOR_PIXFORMAT_JPEG) {
            jpeg = f;
        } else if (f->format == ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY && (!yuv || f->fps > yuv->fps)) {
            yuv = f;
        }
    }
    return jpeg ? jpeg : yuv;
}

/* ---------- 캡처 구성 (해상도마다 다시 만든다) ---------- */

/**
 * 캡처를 멈추고 해상도에 묶인 자원(DVP 컨트롤러, 프레임 버퍼, JPEG 버퍼)을 모두 놓는다.
 * cam_lock을 잡은 상태에서 부른다.
 */
static void pipeline_stop(void)
{
    int off = 0;
    esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &off);
    if (s_ctlr) {
        esp_cam_ctlr_stop(s_ctlr);
        esp_cam_ctlr_disable(s_ctlr);
        esp_cam_ctlr_del(s_ctlr);
        s_ctlr = NULL;
    }
    xQueueReset(s_ready_q);
    xQueueReset(s_free_q);

    // 읽는 쪽이 이미 놓은 버퍼를 복사하지 않도록 최신 프레임부터 비운다.
    // 비어 있는 동안 cam_get_jpeg()는 새 프레임이 올 때까지 기다린다.
    pthread_mutex_lock(&s_lock);
    s_latest.buf = NULL;
    s_latest.len = 0;
    pthread_mutex_unlock(&s_lock);

    for (int i = 0; i < CAM_FB_COUNT; i++) {
        free(s_fbs[i]);
        s_fbs[i] = NULL;
    }
    free(s_drop_buf);
    s_drop_buf = NULL;
    for (int i = 0; i < 2; i++) {
        free(s_jpg_bufs[i]);
        s_jpg_bufs[i] = NULL;
    }
    s_jpg_spare = NULL;
    s_jpg_cap = 0;
}

static esp_err_t hw_encoder_setup(void)
{
    if (s_enc == NULL) {
        jpeg_encode_engine_cfg_t eng_cfg = {
            .intr_priority = 0,
            .timeout_ms = 200,
        };
        ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng_cfg, &s_enc), TAG, "jpeg_new_encoder_engine");
    }

    s_enc_cfg = (jpeg_encode_cfg_t) {
        .width = s_fmt.width,
        .height = s_fmt.height,
        .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = find_param("quality")->value,
    };

    jpeg_encode_memory_alloc_cfg_t mem_cfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    size_t want = s_fmt.width * s_fmt.height;   // 화소당 1 byte면 충분하다.
    size_t got1 = 0;
    size_t got2 = 0;
    s_jpg_bufs[0] = jpeg_alloc_encoder_mem(want, &mem_cfg, &got1);
    s_jpg_bufs[1] = jpeg_alloc_encoder_mem(want, &mem_cfg, &got2);
    ESP_RETURN_ON_FALSE(s_jpg_bufs[0] && s_jpg_bufs[1], ESP_ERR_NO_MEM, TAG, "jpeg output buffer");
    s_jpg_spare = s_jpg_bufs[0];
    s_jpg_cap = got1 < got2 ? got1 : got2;
    return ESP_OK;
}

/**
 * 센서를 fmt로 설정하고 DVP 컨트롤러와 버퍼를 만든 뒤 캡처를 시작한다.
 * cam_lock을 잡은 상태에서 부른다. 실패하면 pipeline_stop()으로 정리해야 한다.
 */
static esp_err_t pipeline_start(const esp_cam_sensor_format_t *fmt)
{
    if (fmt->xclk != s_xclk_hz) {
        ESP_LOGI(TAG, "Switching XCLK to %d Hz for this format", fmt->xclk);
        ESP_RETURN_ON_ERROR(xclk_start(fmt->xclk), TAG, "xclk restart");
    }
    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(s_sensor, fmt), TAG, "set_format");

    bool jpeg = fmt->format == ESP_CAM_SENSOR_PIXFORMAT_JPEG;
    bool first = (s_cur == NULL) || (jpeg != s_sensor_jpeg);
    s_cur = fmt;
    s_fmt = *fmt;
    s_sensor_jpeg = jpeg;
    ESP_LOGI(TAG, "Format in use: %s (%s)", s_fmt.name, s_sensor_jpeg ? "sensor JPEG" : "HW JPEG encoder");

    if (first) {
        params_init();
    } else {
        params_reapply();
    }

    // DVP 컨트롤러. XCLK는 LEDC가 만들므로 external_xtal로 둔다.
    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2, CAM_PIN_D3,
            CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7,
        },
        .vsync_io = CAM_PIN_VSYNC,
        .de_io = CAM_PIN_HREF,
        .pclk_io = CAM_PIN_PCLK,
        .xclk_io = GPIO_NUM_NC,
    };
    cam_ctlr_color_t color = s_sensor_jpeg ? CAM_CTLR_COLOR_RGB565 : CAM_CTLR_COLOR_YUV422_UYVY;
    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = s_fmt.width,
        .v_res = s_fmt.height,
        .input_data_color_type = color,     // JPEG 모드에서는 무시된다.
        .output_data_color_type = color,
        .cam_data_width = 8,
        .bk_buffer_dis = 1,
        .pic_format_jpeg = s_sensor_jpeg,
        .external_xtal = 1,
        .dma_burst_size = 128,
        .xclk_freq = s_fmt.xclk,
        .pin = &pin_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_cam_new_dvp_ctlr(&dvp_cfg, &s_ctlr), TAG, "esp_cam_new_dvp_ctlr");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_get_frame_buffer_len(s_ctlr, &s_fb_size), TAG, "get_frame_buffer_len");

    // PSRAM 프레임 버퍼
    for (int i = 0; i < CAM_FB_COUNT; i++) {
        s_fbs[i] = esp_cam_ctlr_alloc_buffer(s_ctlr, s_fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(s_fbs[i], ESP_ERR_NO_MEM, TAG, "frame buffer %d", i);
        xQueueSend(s_free_q, &s_fbs[i], 0);
    }
    s_drop_buf = esp_cam_ctlr_alloc_buffer(s_ctlr, s_fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_drop_buf, ESP_ERR_NO_MEM, TAG, "drop buffer");
    ESP_LOGI(TAG, "%d frame buffers x %u bytes in PSRAM", CAM_FB_COUNT, (unsigned)s_fb_size);

    if (!s_sensor_jpeg) {
        ESP_RETURN_ON_ERROR(hw_encoder_setup(), TAG, "hw_encoder_setup");
    }

    // 캡처 시작. 컨트롤러를 먼저 켜고 센서 출력을 켠다.
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_register_event_callbacks(s_ctlr, &cbs, NULL), TAG, "register callbacks");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(s_ctlr), TAG, "ctlr enable");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_start(s_ctlr), TAG, "ctlr start");

    int on = 1;
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &on), TAG, "stream on");
    return ESP_OK;
}

/** cam_task 안에서 해상도 전환 요청을 처리한다. 실패하면 이전 포맷으로 되돌린다. */
static void handle_switch_request(void)
{
    const esp_cam_sensor_format_t *fmt = s_req_fmt;
    const esp_cam_sensor_format_t *prev = s_cur;
    s_req_fmt = NULL;

    int64_t t0 = esp_timer_get_time();
    cam_lock();
    pipeline_stop();
    esp_err_t err = pipeline_start(fmt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Switch to %s failed (%s), restoring %s", fmt->name, esp_err_to_name(err), prev->name);
        pipeline_stop();
        if (pipeline_start(prev) != ESP_OK) {
            ESP_LOGE(TAG, "Restore failed, camera stopped");
            pipeline_stop();
            s_running = false;
        }
    } else {
        s_running = true;
    }
    cam_unlock();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Resolution switched to %ux%u @ %u fps in %lld ms", s_fmt.width, s_fmt.height,
                 (unsigned)s_fmt.fps, (long long)((esp_timer_get_time() - t0) / 1000));
    }
    s_req_err = err;
    xSemaphoreGive(s_req_done);
}

/* ---------- 프레임 처리 태스크 ---------- */

/** 새 JPEG을 최신 프레임으로 등록하고, 밀려난 이전 버퍼를 돌려준다. */
static uint8_t *publish_jpeg(uint8_t *buf, size_t len)
{
    pthread_mutex_lock(&s_lock);
    uint8_t *old = s_latest.buf;
    s_latest.buf = buf;
    s_latest.len = len;
    s_seq++;
    if (s_seq == 0) {
        s_seq = 1;  // 0은 "아직 받은 프레임 없음"을 뜻하므로 건너뛴다.
    }
    pthread_cond_broadcast(&s_cond);
    pthread_mutex_unlock(&s_lock);
    return old;
}

static void cam_task(void *arg)
{
    frame_t f;
    int64_t t_start = esp_timer_get_time();
    int64_t t_last = t_start;   // 마지막으로 프레임을 받은 시각
    uint32_t n_start = s_frames;

    while (true) {
        if (s_req_fmt) {
            handle_switch_request();
            t_start = t_last = esp_timer_get_time();
            n_start = s_frames;
            s_fps = 0;
        }

        if (xQueueReceive(s_ready_q, &f, pdMS_TO_TICKS(100)) != pdTRUE) {
            int64_t now = esp_timer_get_time();
            if (s_running && now - t_last >= 3000000) {
                ESP_LOGW(TAG, "No frame for 3 s (bad=%" PRIu32 ", dropped=%" PRIu32 ")", s_bad, s_dropped);
                s_fps = 0;
                t_last = now;
            }
            continue;
        }
        t_last = esp_timer_get_time();

        if (s_sensor_jpeg) {
            // 센서 JPEG: DMA 버퍼를 그대로 최신 프레임으로 쓰고 이전 버퍼를 DMA에 돌려준다.
            uint8_t *old = publish_jpeg(f.buf, f.len);
            if (old) {
                xQueueSend(s_free_q, &old, 0);
            }
        } else {
            // YUV422: 하드웨어로 JPEG 압축한 뒤 YUV 버퍼는 바로 DMA에 돌려준다.
            uint32_t jpg_len = 0;
            xSemaphoreTake(s_enc_mutex, portMAX_DELAY);
            esp_err_t err = jpeg_encoder_process(s_enc, &s_enc_cfg, f.buf, s_fmt.width * s_fmt.height * 2,
                                                 s_jpg_spare, s_jpg_cap, &jpg_len);
            xSemaphoreGive(s_enc_mutex);
            xQueueSend(s_free_q, &f.buf, 0);
            if (err != ESP_OK || jpg_len == 0) {
                s_bad++;
                if ((s_bad % 50) == 1) {
                    ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
                }
                continue;
            }
            uint8_t *old = publish_jpeg(s_jpg_spare, jpg_len);
            s_jpg_spare = old ? old : s_jpg_bufs[1];
        }
        s_frames++;

        int64_t now = esp_timer_get_time();
        if (now - t_start >= 1000000) {
            s_fps = (float)(s_frames - n_start) * 1e6f / (float)(now - t_start);
            t_start = now;
            n_start = s_frames;
        }
    }
}

/*
 * ESP32-S31(ESP-IDF v6.1)의 esp_restart()는 CPU만 리셋하고 주변장치는 리셋하지 않는다
 * (esp_system/port/soc/esp32s31/system_internal.c, IDF-14841).
 * 캡처를 켠 채 재시작하면 DVP 컨트롤러와 AXI GDMA가 재부팅 중에도 PSRAM에 프레임을 계속 써서
 * 다음 부팅의 PSRAM 메모리 테스트가 실패하고(abort 후 한 번 더 재부팅), 그 뒤에는 프레임이 들어오지 않는다.
 * 그래서 esp_restart()가 CPU를 리셋하기 전에 센서 출력, DVP/DMA, JPEG 인코더를 멈춘다.
 * (panic·WDT 리셋은 셧다운 핸들러를 거치지 않으므로 여기서 막지 못한다.)
 */
static void cam_shutdown(void)
{
    // 해상도 전환 중이면 끝날 때까지 기다린다. 재부팅 직전이므로 잠금은 풀지 않는다.
    xSemaphoreTakeRecursive(s_cam_lock, pdMS_TO_TICKS(2000));
    s_running = false;

    if (s_ctlr) {
        int off = 0;
        esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &off);
        esp_cam_ctlr_stop(s_ctlr);
        esp_cam_ctlr_disable(s_ctlr);
    }

    // 인코딩 중이면 끝날 때까지 기다리고, 돌려주지 않아 다음 인코딩을 막는다.
    if (s_enc) {
        xSemaphoreTake(s_enc_mutex, pdMS_TO_TICKS(500));
    }
    esp_rom_delay_us(1000);     // 진행 중이던 DMA 버스트가 끝날 시간
}

/* ---------- 공개 함수 ---------- */

esp_err_t cam_init(void)
{
    pthread_mutex_init(&s_lock, NULL);
    pthread_cond_init(&s_cond, NULL);

    s_free_q = xQueueCreate(CAM_FB_COUNT, sizeof(uint8_t *));
    s_ready_q = xQueueCreate(CAM_FB_COUNT, sizeof(frame_t));
    s_enc_mutex = xSemaphoreCreateMutex();
    s_cam_lock = xSemaphoreCreateRecursiveMutex();
    s_req_lock = xSemaphoreCreateMutex();
    s_req_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_free_q && s_ready_q && s_enc_mutex && s_cam_lock && s_req_lock && s_req_done,
                        ESP_ERR_NO_MEM, TAG, "queue");

    // 1) XCLK. 센서는 XCLK가 있어야 SCCB에 응답하므로 감지보다 먼저 켠다 (Korvo-1 BSP와 같이 LEDC 사용).
    ESP_RETURN_ON_ERROR(xclk_start(CAM_DETECT_XCLK_HZ), TAG, "xclk start");
    vTaskDelay(pdMS_TO_TICKS(20));

    // 2) 센서 감지와 지원 포맷 조회
    ESP_RETURN_ON_ERROR(sensor_detect(), TAG, "sensor_detect");
    ESP_RETURN_ON_ERROR(esp_cam_sensor_query_format(s_sensor, &s_fmts), TAG, "query_format");
    for (uint32_t i = 0; i < s_fmts.count; i++) {
        ESP_LOGI(TAG, "  fmt[%" PRIu32 "] %s", i, s_fmts.format_array[i].name);
    }

    const esp_cam_sensor_format_t *fmt = pick_format(CAM_WIDTH, CAM_HEIGHT);
    ESP_RETURN_ON_FALSE(fmt, ESP_ERR_NOT_SUPPORTED, TAG,
                        "%s has no JPEG or YUV422(UYVY) format at %dx%d. Enable one in menuconfig "
                        "(Component config > Espressif Camera Sensors Configurations) or change the resolution.",
                        esp_cam_sensor_get_name(s_sensor), CAM_WIDTH, CAM_HEIGHT);

    // 3) 센서 포맷, DVP 컨트롤러, 버퍼를 만들고 캡처를 시작한다.
    cam_lock();
    esp_err_t err = pipeline_start(fmt);
    if (err != ESP_OK) {
        pipeline_stop();
    }
    cam_unlock();
    ESP_RETURN_ON_ERROR(err, TAG, "pipeline_start");

    BaseType_t ok = xTaskCreatePinnedToCore(cam_task, "cam_task", 4096, NULL, 6, &s_task, tskNO_AFFINITY);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "cam_task");

    s_running = true;
    ESP_RETURN_ON_ERROR(esp_register_shutdown_handler(cam_shutdown), TAG, "register shutdown handler");
    ESP_LOGI(TAG, "Camera started: %s %ux%u, %s", esp_cam_sensor_get_name(s_sensor), s_fmt.width, s_fmt.height,
             s_sensor_jpeg ? "sensor JPEG" : "YUV422 -> HW JPEG");
    return ESP_OK;
}

esp_err_t cam_set_resolution(uint16_t width, uint16_t height)
{
    ESP_RETURN_ON_FALSE(s_sensor && s_task, ESP_ERR_INVALID_STATE, TAG, "camera not initialized");
    const esp_cam_sensor_format_t *fmt = pick_format(width, height);
    ESP_RETURN_ON_FALSE(fmt, ESP_ERR_NOT_SUPPORTED, TAG, "%ux%u is not supported by %s",
                        width, height, esp_cam_sensor_get_name(s_sensor));
    if (fmt == s_cur && s_running) {
        return ESP_OK;
    }

    xSemaphoreTake(s_req_lock, portMAX_DELAY);
    xSemaphoreTake(s_req_done, 0);  // 시간 초과로 남은 이전 완료 신호를 버린다.
    ESP_LOGI(TAG, "Switching resolution to %ux%u (%s)", width, height, fmt->name);
    s_req_fmt = fmt;
    esp_err_t err = ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(s_req_done, pdMS_TO_TICKS(CAM_SWITCH_TIMEOUT_MS)) == pdTRUE) {
        err = s_req_err;
    }
    xSemaphoreGive(s_req_lock);
    return err;
}

int cam_resolutions_to_json(char *out, size_t size)
{
    int n = snprintf(out, size, "[");
    bool first = true;
    for (uint32_t i = 0; i < s_fmts.count && n < (int)size; i++) {
        const esp_cam_sensor_format_t *f = &s_fmts.format_array[i];
        if (pick_format(f->width, f->height) != f) {
            continue;   // 해상도마다 실제로 쓰는 포맷 하나만 내보낸다.
        }
        n += snprintf(out + n, size - n, "%s{\"width\":%u,\"height\":%u,\"fps\":%u}",
                      first ? "" : ",", f->width, f->height, (unsigned)f->fps);
        first = false;
    }
    if (n < (int)size) {
        n += snprintf(out + n, size - n, "]");
    }
    return n;
}

esp_err_t cam_get_jpeg(uint32_t *seq, uint8_t **buf, size_t *cap, size_t *len, uint32_t timeout_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    esp_err_t ret = ESP_OK;
    pthread_mutex_lock(&s_lock);
    while (s_latest.buf == NULL || (*seq != 0 && s_seq == *seq)) {
        if (pthread_cond_timedwait(&s_cond, &s_lock, &deadline) != 0) {
            ret = ESP_ERR_TIMEOUT;
            break;
        }
    }

    if (ret == ESP_OK) {
        if (*cap < s_latest.len) {
            uint8_t *nb = heap_caps_realloc(*buf, s_latest.len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (nb == NULL) {
                ret = ESP_ERR_NO_MEM;
            } else {
                *buf = nb;
                *cap = s_latest.len;
            }
        }
        if (ret == ESP_OK) {
            memcpy(*buf, s_latest.buf, s_latest.len);
            *len = s_latest.len;
            *seq = s_seq;
        }
    }
    pthread_mutex_unlock(&s_lock);
    return ret;
}

void cam_get_status(cam_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->running = s_running;
    st->sensor = s_sensor ? esp_cam_sensor_get_name(s_sensor) : "none";
    st->format = s_fmt.name ? s_fmt.name : "";
    st->jpeg_source = s_running ? (s_sensor_jpeg ? "sensor" : "hw") : "";
    st->width = s_running ? s_fmt.width : CAM_WIDTH;
    st->height = s_running ? s_fmt.height : CAM_HEIGHT;
    st->sensor_fps = s_running ? s_fmt.fps : 0;
    st->fps = s_fps;
    st->frames = s_frames;
    st->dropped = s_dropped;
    st->bad = s_bad;
    pthread_mutex_lock(&s_lock);
    st->jpeg_size = s_latest.len;
    pthread_mutex_unlock(&s_lock);
}

esp_err_t cam_set_param(const char *name, int value)
{
    ESP_RETURN_ON_FALSE(s_sensor, ESP_ERR_INVALID_STATE, TAG, "sensor not ready");
    cam_param_t *p = find_param(name);
    ESP_RETURN_ON_FALSE(p && p->supported, ESP_ERR_NOT_SUPPORTED, TAG, "param '%s' not supported", name);
    ESP_RETURN_ON_FALSE(value >= p->min && value <= p->max, ESP_ERR_INVALID_ARG, TAG,
                        "%s=%d out of range [%d, %d]", name, value, p->min, p->max);

    esp_err_t err;
    cam_lock();
    if (!s_sensor_jpeg && p->id == ESP_CAM_SENSOR_JPEG_QUALITY) {
        s_enc_cfg.image_quality = value;    // 다음 프레임부터 적용된다.
        err = ESP_OK;
    } else {
        err = esp_cam_sensor_set_para_value(s_sensor, p->id, &value, sizeof(value));
    }
    if (err == ESP_OK) {
        p->value = value;
    }
    cam_unlock();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Set %s = %d", name, value);
    } else {
        ESP_LOGW(TAG, "Set %s = %d failed: %s", name, value, esp_err_to_name(err));
    }
    return err;
}

int cam_params_to_json(char *out, size_t size)
{
    int n = snprintf(out, size, "{");
    bool first = true;
    for (size_t i = 0; i < PARAM_COUNT && n < (int)size; i++) {
        const cam_param_t *p = &s_params[i];
        if (!p->supported) {
            continue;
        }
        n += snprintf(out + n, size - n, "%s\"%s\":{\"value\":%d,\"min\":%d,\"max\":%d}",
                      first ? "" : ",", p->name, p->value, p->min, p->max);
        first = false;
    }
    if (n < (int)size) {
        n += snprintf(out + n, size - n, "}");
    }
    return n;
}
