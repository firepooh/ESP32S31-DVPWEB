#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 콘솔(UART0 또는 USB Serial/JTAG) 드라이버를 설치하고 표준 입출력을 드라이버로 연결한다. */
esp_err_t console_init(void);

/**
 * 콘솔에서 한 줄을 입력받는다 (CR, LF, CRLF 모두 줄 끝으로 처리).
 * mask가 true이면 입력 문자 대신 '*'를 표시한다.
 * reprompt_ms 동안 아무 입력이 없으면 안내 문구를 다시 출력한다 (0이면 다시 출력하지 않는다).
 */
int console_read_line(const char *prompt, char *buf, size_t len, bool mask, uint32_t reprompt_ms);

/** 유지보수 명령(help, status, wifi, wifi_reset, restart)을 처리하는 태스크를 시작한다. */
void console_start_commands(void);

#ifdef __cplusplus
}
#endif
