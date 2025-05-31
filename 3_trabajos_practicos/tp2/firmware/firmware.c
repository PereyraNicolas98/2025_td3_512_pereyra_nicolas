#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

//Cola para mandar los valores de Temperatura
QueueHandle_t queue_temp;

uint16_t lectura;
float temp, adc_volt, adc_temp;

//Constante de conversion
const float conversion_factor = 3.3f / (1 << 12);

void adc_irq_handler(void) {
    // Variable de verificación de cambio tarea
    BaseType_t to_higher_priority_task = pdFALSE;
    // Deshabilitación de la interrupción y del ADC
    adc_irq_set_enabled(false);
    adc_run(false);
    uint16_t medida = adc_fifo_get();
    adc_fifo_drain();
    xQueueSendFromISR(queue_temp, &medida, &to_higher_priority_task);
    portYIELD_FROM_ISR(to_higher_priority_task);
}

void task_medir(void *params)
{
    while(1)
    {
        adc_irq_set_enabled(true);
        adc_run(true);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_mostrar(void *params)
{
    while(1)
    {
        xQueueReceive(queue_temp, &lectura, portMAX_DELAY);
        adc_volt= (float)(lectura * conversion_factor);
        adc_temp= 27 - (adc_volt - 0.706)/0.001721;
        printf("Temperatura: %.2f°C\n", adc_temp);
    }

}

int main()
{
    stdio_init_all();
    
    //Inicialización de la cola
    queue_temp= xQueueCreate(1, sizeof(uint16_t));

    //Inicialización de ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    //Configuración de interrupción y FIFO
    adc_fifo_setup(true, false, 1, false, false);
    adc_irq_set_enabled(true);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    xTaskCreate(task_medir,"Medir", configMINIMAL_STACK_SIZE,NULL,1,NULL);
    xTaskCreate(task_mostrar,"Mostrar", configMINIMAL_STACK_SIZE*2,NULL,2,NULL);

    vTaskStartScheduler();
    while(1);
}
