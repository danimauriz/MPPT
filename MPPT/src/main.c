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
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "esp_adc/adc_cali_scheme.h"



#define PWM_FREQ       10000
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_GPIO       25

static const char *TAG = "WebSocket Server"; // TAG for debug

httpd_handle_t server = NULL;

adc_oneshot_unit_handle_t adc_handle = NULL;

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

// Variables compartidas
volatile float current = 0;
volatile float panel_voltage = 0.0;
volatile float battery_voltage = 0.0;
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

void config_ADC() {

    adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_12, // or you can use ADC_BITWIDTH_DEFAULT here
    .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &config));
}


float read_current() {
    int adc_current_sense = 0;
    esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &adc_current_sense);
    if (err != ESP_OK) {
        printf("ACD current sensor read failed: %s", esp_err_to_name(err));
    }
    printf(" ADC_CHANNEL_6 (GPIO 34) ADC input current _sensor:   %d \n", adc_current_sense);



    return (float)adc_current_sense;
}

float read_panel_voltage() {
    int adc_panel_voltage_sense = 0;
    float voltage = 0.0;
    esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_panel_voltage_sense);
    if (err != ESP_OK) {
        printf("ACD panel voltage read failed: %s", esp_err_to_name(err));
    }
    printf(" ADC_CHANNEL_7 (GPIO 35) ADC input panel voltage:   %d \n", adc_panel_voltage_sense);
    voltage = adc_panel_voltage_sense * 45 / 4096;
    printf(" Input panel voltage:   %.2f \n", voltage);

    return voltage;
}

float read_battery_voltage() {
    int adc_battery_voltage_sense = 0;
    float voltage = 0.0;
    esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &adc_battery_voltage_sense);
    if (err != ESP_OK) {
        printf("ACD battery voltage read failed: %s", esp_err_to_name(err));
    }
    printf(" ADC_CHANNEL_4 (GPIO 32) ADC input battery voltage:   %d \n", adc_battery_voltage_sense);
    voltage = adc_battery_voltage_sense * 16.5 / 4096;
    printf(" Input battery voltage:   %.2f \n", voltage);

    return voltage;
}

// --- Tarea en núcleo 0: Lee sensores y ajusta PWM ---
void sensor_task(void *pvParameters) {
    while (1) {
        /*
        float read = esp_random() % 1000 / 10.0f; // Simulación sensor
        float result = read * 1.5f;               // Simulación algoritmo

        int duty = (int)(result * 10);
        if (duty > 1023) duty = 1023;
        */
        

        float current_sense = read_current();
        float panel_voltage_sense = read_panel_voltage();
        float battery_voltage_sense = read_battery_voltage();

        taskENTER_CRITICAL(&data_mux);
        current = current_sense;
        panel_voltage = panel_voltage_sense;
        battery_voltage = battery_voltage_sense;
        taskEXIT_CRITICAL(&data_mux);

        int duty = 1023*0.33;

        printf("Duty: %i\n", duty);

         

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void ws_async_send(void *arg)
{
    httpd_ws_frame_t ws_pkt;
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    
    char buff[32];
    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%f",current);
    
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK) {
        return;
    }

    for (int i = 0; i < fds; i++) {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(hd, client_fds[i], &ws_pkt);
        }
    }
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    if (resp_arg == NULL) {
        return ESP_ERR_NO_MEM;
    }
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    esp_err_t ret = httpd_queue_work(handle, ws_async_send, resp_arg);
    if (ret != ESP_OK) {
        free(resp_arg);
    }
    return ret;
}

// --- Web server handler ---
esp_err_t web_handler(httpd_req_t *req) {
    char resp[1024]; // Tamanio del buffer de respuesta

    taskENTER_CRITICAL(&data_mux);
    float c = current;
    float p = panel_voltage;
    float b = battery_voltage;
    int d = duty_cycle;
    taskEXIT_CRITICAL(&data_mux);

    /*
    snprintf(resp, sizeof(resp),
             "<html><body><h1>ESP32 Monitor</h1>"
             "Current_Sensor: %.2f<br>Panel_Voltage: %.2f<br>Battery_Voltage: %.2f<br>Duty Cycle: %d<br>"
             "</body></html>", c, p, b, d);
    */
    snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>ESP32 Monitor</title></head><body>"
        "<h1>ESP32 Monitor</h1>"
        "<p>Current_Sensor: <span id='current'>%.2f</span></p>"
        "<p>Panel_Voltage: <span id='panel'>%.2f</span></p>"
        "<p>Battery_Voltage: <span id='battery'>%.2f</span></p>"
        "<script>"
        "var ws = new WebSocket('ws://' + location.host + '/ws');"
        "ws.onmessage = function(event) {"
        "  var data = JSON.parse(event.data);"
        "  document.getElementById('current').textContent = data.current.toFixed(2);"
        "  document.getElementById('panel').textContent = data.panel.toFixed(2);"
        "  document.getElementById('battery').textContent = data.battery.toFixed(2);"
        "};"
        "</script></body></html>",
        c, p, b  // Ahora usamos las variables para mostrar los valores iniciales
    );

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
        printf("Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "toggle") == 0)
    {
        free(buf);
        return trigger_async_send(req->handle, req);
    }

    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    }
    free(buf);
    return ret;
}

// --- Inicia servidor web ---
void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    //httpd_handle_t server = NULL;
    


    
    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = web_handler,
        .user_ctx = NULL
    };
    

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handle_ws_req,
        .user_ctx = NULL,
        .is_websocket = true
    };

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri);
        httpd_register_uri_handler(server, &ws);
    }
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

// --- Tarea en núcleo 1: Envío periódico por websockets ---
void websocket_broadcast_task(void *pvParameters) {
    //Espera a que este inicializado el webserver
    while (server == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (1) {
        httpd_ws_frame_t ws_pkt;
        char buff[64];
        memset(buff, 0, sizeof(buff));

        taskENTER_CRITICAL(&data_mux);
        float c = current;
        float p = panel_voltage;
        float b = battery_voltage;
        taskEXIT_CRITICAL(&data_mux);

        sprintf(buff, "{\"current\": %.2f, \"panel\": %.2f, \"battery\": %.2f}", c, p, b);


        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t *)buff;
        ws_pkt.len = strlen(buff);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
        int client_fds[max_clients];
        size_t fds = max_clients;

        if (httpd_get_client_list(server, &fds, client_fds) == ESP_OK) {
            for (int i = 0; i < fds; i++) {
                if (httpd_ws_get_fd_info(server, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                    httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // cada 1 segundo
    }
}

// --- app_main ---
void app_main(void) {
    nvs_flash_init(); // Necesario para WiFi
    init_pwm();
    config_ADC();

    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(web_task,    "WebTask",    4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(websocket_broadcast_task, "WebSocketBroadcast", 4096, NULL, 1, NULL, 1);
}