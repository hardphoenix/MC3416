#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_log.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "MC3416.h"


/// @brief LED Status ///////////////////////////////
typedef enum
{
    LED_ON=1,
    LED_OFF=0,
}LED_STATE_E;

void GPIO_INIT(void)
{
    gpio_config_t gpio;

    gpio.intr_type= GPIO_INTR_DISABLE;
    gpio.mode= GPIO_MODE_OUTPUT;
    gpio.pin_bit_mask = ((1ULL<<2));
    gpio.pull_down_en =0;
    gpio.pull_up_en=0;
    gpio_set_direction(GPIO_NUM_2,GPIO_MODE_OUTPUT);
    gpio_config(&gpio);
}

void GPIO_LED(LED_STATE_E LED_STAT)
{
    gpio_set_level(GPIO_NUM_2,(uint32_t)LED_STAT);
}
///////////////////////////////////////////

void MC3416_Task(void)
{
    GPIO_INIT();
    /*config and init mc34xx-----------*/
    MC34xx_ChipParam_t mc34xx_config = {0};
    mc34xx_config.g_range=g_range_4g;
    mc34xx_config.sample_rate=sample_rate_1024;
    MC34xx_Init(&mc34xx_config);    //init mc3416
    /*---------------------------------*/
    
    static float x_axis,y_axis,z_axis;

    while (1)
    {
        MC34xx_Get_XYZ_Float(&x_axis,&y_axis,&z_axis);
        printf("\r\nX=%1.4f, Y=%1.4f, Z=%1.4f\r\n",x_axis, y_axis, z_axis);
        GPIO_LED(LED_ON);
        vTaskDelay(pdMS_TO_TICKS(500));
        GPIO_LED(LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(MC3416_Task,"MC3416Task",1024*4,NULL,5,NULL);
}
