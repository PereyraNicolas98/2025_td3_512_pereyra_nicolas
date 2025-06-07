#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/i2c.h"
#include "lcd.h"
#include "helper.h"
#include "semphr.h"

#define CUENTA_MAX 4096

//Lectura del pulso pata 20(GPIO15)
#define GPIO_PULSO 15

//PWM pata 16(GPIO12)
#define GPIO_PWM 12
#define PWM_FREC 5000

//Declaro el semáforo
SemaphoreHandle_t semaforo_cuenta;

uint32_t cuenta_pulso;
float frec;

void lectura_pulso(void *params)
{
    while(true)
    {
        if(gpio_get(GPIO_PULSO))
        {
            xSemaphoreGive(semaforo_cuenta);
            while(gpio_get(GPIO_PULSO));
        }
    }
}

void escritura_consola(void *params)
{
    char msg[MAX_CHARS]={0};
    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        cuenta_pulso=uxSemaphoreGetCount(semaforo_cuenta);
        frec= cuenta_pulso * (1000/(float)250);
        if(cuenta_pulso==CUENTA_MAX)
            snprintf(msg, MAX_CHARS, "Max f:%.1fHz", frec);
        else
            snprintf(msg, MAX_CHARS, "%.2fHz", frec);
        xQueueReset(semaforo_cuenta);
        printf("Frecuencia: %s\n", msg);
    }
}

int main()
{
    stdio_init_all();
    pwm_user_init(GPIO_PWM,PWM_FREC);

    gpio_init(GPIO_PULSO);
    gpio_set_dir(GPIO_PULSO, GPIO_IN);
    gpio_pull_down(GPIO_PULSO);
    
    //Inicialización del semáforo
    semaforo_cuenta=xSemaphoreCreateCounting(CUENTA_MAX, 0);

    //Inicialización de tareas
    xTaskCreate(escritura_consola, "escritura_consola", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL);
    xTaskCreate(lectura_pulso, "lectura_pulso", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}
