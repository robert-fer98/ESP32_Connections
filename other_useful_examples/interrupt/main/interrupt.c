#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define INPUT_PIN 32
#define LED_PIN 33

int state = 0;
bool ledState = false;
bool update = false;
QueueHandle_t interputQueue;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{

    ledState = !ledState;
    update = true;
    //int pinNumber = (int)args;
    //xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void LED_Control_Task(void *params)
{
    int pinNumber, count = 0;
    while (true)
    {
        vTaskDelay(1);
       /* if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count++, gpio_get_level(INPUT_PIN));
            ledState = !ledState;
            if (!ledState) gpio_set_level(LED_PIN, 1);
            else gpio_set_level(LED_PIN, 0);
        }*/
        if (update) {
            if (!ledState) {
                //ledState = true;
                gpio_set_level(LED_PIN, 1);
            }
            else {
                //ledState = false;
                gpio_set_level(LED_PIN, 0);
            }
            update = false;
        }
    }
}

void app_main()
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    // arguments - task, task name, stack size, parameter, priority of the task, handle of the task (pointer)
    xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 2, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
}

/*
void app_main() {
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    while(1) {
        if (gpio_get_level(INPUT_PIN) == 1) {
            printf("BUTTON PRESS\n");
        }
        //vTaskDelay(1 / portTICK_PERIOD_MS); // we have to satisfy the watchdog
    }
}*/