#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/i2c.h"
#include "lcd.h"
#include "helper.h"
#include "semphr.h"

// I2C
#define I2C_PORT i2c0

//SDA pata 6(GPIO4), SCL pata 7(GPIO5) y Frecuencia 400KHz
#define I2C_SDA 4
#define I2C_SCL 5
#define I2C 400*1000

//LCD
#define LCD_DIR 0x3F
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

void escritura_LCD(void *params)
{
    char msg[MAX_CHARS]={0};
    while(true)
    {
        gpio_set_irq_enabled(GPIO_PULSO, GPIO_IRQ_EDGE_RISE, true);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_irq_enabled(GPIO_PULSO, GPIO_IRQ_EDGE_RISE, false);
        cuenta_pulso=uxSemaphoreGetCount(semaforo_cuenta);
        frec= cuenta_pulso * (1000/(float)250);
        if(cuenta_pulso==CUENTA_MAX)
            snprintf(msg, MAX_CHARS, "Max f:%.1fHz", frec);
        else
            snprintf(msg, MAX_CHARS, "%.2fHz", frec);
        lcd_clear();
        lcd_set_cursor(0,0);
        lcd_string("Frecuencia:");
        lcd_set_cursor(1,0);
        lcd_string(msg);
        xQueueReset(semaforo_cuenta);
    }
}

void lectura_pulso_irq(uint gpio, uint32_t event_mask) 
{
    BaseType_t to_higher_priority = pdTRUE;
    if (event_mask & GPIO_IRQ_EDGE_RISE) 
    {
        xSemaphoreGiveFromISR(semaforo_cuenta, &to_higher_priority);
        portYIELD_FROM_ISR(to_higher_priority);
    }
}

int main()
{
    stdio_init_all();
    pwm_user_init(GPIO_PWM,PWM_FREC);

    gpio_init(GPIO_PULSO);
    gpio_set_dir(GPIO_PULSO, GPIO_IN);
    gpio_pull_down(GPIO_PULSO);
    
    //Inicialización de la interrupción
    gpio_set_irq_enabled_with_callback(GPIO_PULSO, GPIO_IRQ_EDGE_RISE, true, lectura_pulso_irq);

    //Inicialización de I2C
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    //Inicialización de LCD
    lcd_init(I2C_PORT, LCD_DIR);

    //Inicialización del semáforo
    semaforo_cuenta=xSemaphoreCreateCounting(CUENTA_MAX, 0);

    //Inicialización de tareas
    xTaskCreate(escritura_LCD, "escritura_LCD", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL); 
    
    vTaskStartScheduler();
    while (true);
}
