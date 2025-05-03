#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"


#define PWM_PIN       23      // Pin GPIO de salida PWM
#define PWM_FREQ_HZ   5000    // Frecuencia de 5kHz
#define PWM_DUTY_RES  LEDC_TIMER_8_BIT  // Resolución de 8 bits (0-255)
#define PWM_CHANNEL   LEDC_CHANNEL_0
#define PWM_TIMER     LEDC_TIMER_0
#define PWM_MODE      LEDC_HIGH_SPEED_MODE

esp_err_t create_task(void);
void adquisicion_datos(void *pvParameters);
void servidor_web(void *pvParameters);

void app_main(void)
{
    // Configurar el temporizador
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_DUTY_RES,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        printf("Error configuring timer: %s\n", esp_err_to_name(err));
    }
    printf("TIMER CONFIGURADO\n");

    // Configurar el canal
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_PIN,
        .duty           = 0, // Lo cambiamos después
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    create_task();

    // Calcular duty del 33% para resolución de 8 bits
    uint32_t max_duty = (1 << 8) - 1; // 255
    printf("\nMax duty %lu\n", max_duty);
    uint32_t duty_33 = (max_duty * 33) / 100; // ≈84
    printf("Duty actual = %lu\n", duty_33);
    // Establecer el duty fijo
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_33);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);


}

esp_err_t create_task(void)
{   
    //static uint8_t ucParameterToPass;
    //TaskHandle_t xHandle = NULL;

    xTaskCreatePinnedToCore(
        adquisicion_datos,
        "adquisicion_datos",
        1024,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        servidor_web,
        "servidor_web",
        1024,
        NULL,
        1,
        NULL,
        1
    );
    
    return ESP_OK;
}

void adquisicion_datos(void *pvParameters)
{   
    printf("Iniciadas Tareas Core 0\n");

    // Nada más que hacer: duty fijo
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
    
}

void servidor_web(void *pvParameters)
{
    printf("Iniciadas Tareas Core 1\n");

    while(1)
    {
        printf("Tarea Core 1\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}