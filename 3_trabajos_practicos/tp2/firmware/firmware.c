#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/adc.h"

//Cola para mandar los valores de Temperatura
QueueHandle_t queue_temp;

uint16_t lectura;
float temp, adc_volt, adc_temp;

//Constante de conversion
const float conversion_factor = 3.3f / (1 << 12);

void task_medir(void *params)
{
    while(1)
    {
        lectura=adc_read();
        adc_volt= (float)(lectura * conversion_factor);
        adc_temp= 27 - (adc_volt - 0.706)/0.001721;
        xQueueSend(queue_temp, &adc_temp, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_mostrar(void *params)
{
    while(1)
    {
        xQueueReceive(queue_temp, &temp, portMAX_DELAY);
        printf("Temperatura: %.2f°C\n", temp);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

int main()
{
    stdio_init_all();
    
    //Inicialización de la cola
    queue_temp= xQueueCreate(1, sizeof(float));

    //Inicialización de ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    xTaskCreate(task_medir,"Medir", configMINIMAL_STACK_SIZE*2,NULL,1,NULL);
    xTaskCreate(task_mostrar,"Mostrar", configMINIMAL_STACK_SIZE*2,NULL,1,NULL);

    vTaskStartScheduler();
    while(1);
}
