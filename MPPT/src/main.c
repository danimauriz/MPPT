#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"


#define PWM_FREQ       5000
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_GPIO       23

// Variables compartidas
float sensor_value = 0;
float algorithm_result = 0;
int duty_cycle = 0;

// Mutex para sincronización
portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

// --- PWM Init ---
void init_pwm() {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = PWM_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

// --- Tarea en núcleo 0: Lee sensores y ajusta PWM ---
void sensor_task(void *pvParameters) {
    while (1) {
        float read = esp_random() % 1000 / 10.0f; // Simulación sensor
        float result = read * 1.5f;               // Simulación algoritmo

        int duty = (int)(result * 10);
        if (duty > 1023) duty = 1023;

        taskENTER_CRITICAL(&data_mux);
        sensor_value = read;
        algorithm_result = result;
        duty_cycle = duty;
        taskEXIT_CRITICAL(&data_mux);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- Web server handler ---
esp_err_t web_handler(httpd_req_t *req) {
    char resp[256];

    taskENTER_CRITICAL(&data_mux);
    float s = sensor_value;
    float a = algorithm_result;
    int d = duty_cycle;
    taskEXIT_CRITICAL(&data_mux);

    snprintf(resp, sizeof(resp),
             "<html><body><h1>ESP32 Monitor</h1>"
             "Sensor: %.2f<br>Algoritmo: %.2f<br>Duty Cycle: %d<br>"
             "</body></html>", s, a, d);

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// --- Inicia servidor web ---
void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    httpd_start(&server, &config);

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = web_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri);
}

// --- Init WiFi en modo AP ---
void init_wifi_ap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = strlen("ESP32_AP"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();
}

// --- Tarea en núcleo 1: Servidor web ---
void web_task(void *pvParameters) {
    init_wifi_ap();
    start_webserver();
    vTaskDelete(NULL);  // Servidor ya corre, eliminar tarea
}

// --- app_main ---
void app_main(void) {
    nvs_flash_init(); // Necesario para WiFi
    init_pwm();

    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(web_task,    "WebTask",    4096, NULL, 1, NULL, 1);
}