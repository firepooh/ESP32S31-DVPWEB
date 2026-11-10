#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#if CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#error "Console must be UART or USB Serial/JTAG"
#endif
#include "camera.h"
#include "console.h"
#include "web_server.h"
#include "wifi_sta.h"

static const char *TAG = "console";
static bool s_prev_cr;

/*
 * 콘솔 입력은 menuconfig의 콘솔 설정을 따른다.
 * Korvo-1의 USB-UART 브리지(CP210x)는 UART0, 모듈의 USB 포트는 USB Serial/JTAG이다.
 */
esp_err_t console_init(void)
{
#if CONFIG_ESP_CONSOLE_UART
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
        ESP_RETURN_ON_ERROR(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 2048, 0, NULL, 0),
                            TAG, "uart_driver_install");
    }
    // printf/ESP_LOG 출력도 드라이버를 거치게 해서 입력 처리와 충돌하지 않게 한다.
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
#else
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        cfg.rx_buffer_size = 512;
        cfg.tx_buffer_size = 2048;
        ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&cfg), TAG, "usb_serial_jtag_driver_install");
    }
    usb_serial_jtag_vfs_use_driver();
#endif
    setvbuf(stdin, NULL, _IONBF, 0);
    return ESP_OK;
}

static int read_byte(uint8_t *c, TickType_t ticks)
{
#if CONFIG_ESP_CONSOLE_UART
    return uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, c, 1, ticks);
#else
    return usb_serial_jtag_read_bytes(c, 1, ticks);
#endif
}

static void out_str(const char *s)
{
    fputs(s, stdout);
    fflush(stdout);
}

int console_read_line(const char *prompt, char *buf, size_t len, bool mask, uint32_t reprompt_ms)
{
    size_t n = 0;
    int64_t last_activity = esp_timer_get_time();

    if (prompt) {
        out_str(prompt);
    }

    while (true) {
        uint8_t c;
        if (read_byte(&c, pdMS_TO_TICKS(500)) <= 0) {
            int64_t now = esp_timer_get_time();
            if (prompt && reprompt_ms && n == 0 && now - last_activity > (int64_t)reprompt_ms * 1000) {
                out_str("\n");
                out_str(prompt);
                last_activity = now;
            }
            continue;
        }
        last_activity = esp_timer_get_time();

        // CRLF로 들어오면 LF는 건너뛴다.
        if (c == '\n' && s_prev_cr) {
            s_prev_cr = false;
            continue;
        }
        if (c == '\r' || c == '\n') {
            s_prev_cr = (c == '\r');
            out_str("\n");
            buf[n] = '\0';
            return (int)n;
        }
        s_prev_cr = false;

        if (c == 0x08 || c == 0x7f) {   // Backspace / DEL
            if (n > 0) {
                n--;
                out_str("\b \b");
            }
            continue;
        }
        if (c < 0x20) {
            continue;                   // 그 밖의 제어 문자는 무시한다.
        }
        if (n + 1 < len) {
            buf[n++] = (char)c;
            fputc(mask ? '*' : c, stdout);
            fflush(stdout);
        }
    }
}

static void print_status(void)
{
    cam_status_t st;
    char ip[16];

    cam_get_status(&st);
    wifi_sta_get_ip(ip, sizeof(ip));
    printf("camera : %s %s %ux%u, %.1f fps, frames %" PRIu32 ", dropped %" PRIu32 ", bad %" PRIu32 ", jpeg %u B\n",
           st.running ? "running" : "STOPPED", st.sensor, st.width, st.height, st.fps,
           st.frames, st.dropped, st.bad, (unsigned)st.jpeg_size);
    printf("wifi   : SSID \"%s\", %s, ip %s, rssi %d dBm\n", wifi_sta_get_ssid(),
           wifi_sta_is_connected() ? "connected" : "not connected", ip, wifi_sta_get_rssi());
    printf("web    : http://%s/  (stream clients %d)\n", ip, web_server_stream_clients());
    printf("heap   : internal %u, PSRAM %u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    fflush(stdout);
}

static void print_help(void)
{
    out_str("commands:\n"
            "  status      camera / Wi-Fi / web server status\n"
            "  wifi        show saved Wi-Fi SSID and connection\n"
            "  wifi_reset  erase saved Wi-Fi credentials and reboot (asks again)\n"
            "  restart     reboot the board\n");
}

static void command_task(void *arg)
{
    char line[64];

    while (true) {
        int n = console_read_line(NULL, line, sizeof(line), false, 0);
        if (n == 0) {
            continue;
        }
        if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
            print_help();
        } else if (strcmp(line, "status") == 0 || strcmp(line, "wifi") == 0) {
            print_status();
        } else if (strcmp(line, "wifi_reset") == 0) {
            esp_err_t err = wifi_sta_erase_creds();
            printf("Wi-Fi credentials erased (%s). Rebooting...\n", esp_err_to_name(err));
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
        } else if (strcmp(line, "restart") == 0 || strcmp(line, "reboot") == 0) {
            out_str("Rebooting...\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
        } else {
            printf("unknown command: %s (type 'help')\n", line);
            fflush(stdout);
        }
    }
}

void console_start_commands(void)
{
    xTaskCreatePinnedToCore(command_task, "console", 4096, NULL, 3, NULL, tskNO_AFFINITY);
}
