idf_component_register(
    SRCS "app_main.c" "wifi_sta.c" "camera.c" "web_server.c" "console.c"
    INCLUDE_DIRS "."
    EMBED_FILES "www/index.html"
    PRIV_REQUIRES
        esp_driver_cam
        esp_driver_i2c
        esp_driver_jpeg
        esp_driver_ledc
        esp_driver_uart
        esp_driver_usb_serial_jtag
        esp_http_server
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        esp_psram
        lwip
        nvs_flash
        pthread
)
