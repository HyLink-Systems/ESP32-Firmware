#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASS "YourWiFiPassword"

void wifi_init_sta() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

void app_main() {
    esp_log_level_set("wifi", ESP_LOG_NONE);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
}