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

static void my_mc34xx_motion_cb(MC34xx_Flags_Tilt_t flag)
{
    switch ((uint8_t)flag)
    {
    case MC_FLAG_Tilt:
        printf("\n---------------------------->Tilt Detected");
        break;
    case MC_Flag_Flip:
        printf("\n---------------------------->Flip Detected");
        break;
    case MC_Flag_AnyMotion:
        printf("\n---------------------------->Any Motion Detected");
        break;
    case MC_Flag_Shake:
        printf("\n---------------------------->Shake Detected");
        break;
    case MC_Flag_Tilt35:
        printf("\n---------------------------->Tilt35 Detected");
        break;
    case MC_Flag_NewData:
        printf("\n---------------------------->New Data Acquired");
        break;
    }
}

void MC3416_Task(void)
{
    GPIO_INIT();
    /*config and init mc34xx-----------*/
    MC34xx_ChipParam_t mc34xx_config = {0};
    mc34xx_config.g_range=g_range_2g;
    mc34xx_config.sample_rate=sample_rate_1024;
    mc34xx_config.Check_NewDataBit=false;       //Check New Date Acqure
    mc34xx_config.AnyMotion_Threshold=0x000f;       //set minimal threshold
    mc34xx_config.Shake_Threshold=0x000f;           //set minimal threshold
    mc34xx_config.Tilt_Flip_Thrshold=0x000f;        //set minimal threshold - tilt35
    //Active/Disable Motions Detection
    mc34xx_config.MotionCtrl= MC34xx_MakeByte_MotionCtrl(Tilt_Flip_Disable,Latch_En,AnyMotion_En,Shake_En,
                                                        Tilt35_En,Z_AxisOrien_Disable,FilterMotion_Disable,MotionReset_Disable); 
    //Active/Disable Custom Interrupt
    mc34xx_config.INTNCtrl= MC34xx_MakeByte_InterruptCtrl(TILT_INT_Disable,FLIP_INT_Disable,ANYM_INT_Active,SHAKE_INT_Disable,
                                                          TILT_35_INT_Active,AUTO_CLR_Disable,ACQ_INT_Disable);
    MC34xx_Init(&mc34xx_config);    //init MC34xx
    /*---------------------------------*/
    
    static float x_axis,y_axis,z_axis;
    MC34xx_Interrupt_t MC34xx_intn_list;

    while (1)
    {
        /*-----------------------------------*/
        // MC34xx_Get_XYZ_Float(&x_axis,&y_axis,&z_axis);
        // printf("\r\nX=%1.4f, Y=%1.4f, Z=%1.4f\r\n",x_axis, y_axis, z_axis);
        MC34xx_GetStatus_Tilt(my_mc34xx_motion_cb);
        MC34xx_intn_list = MC34xx_CheckSoft_Interrupt();
        printf("\n-------------------------------------\n");
        /*-----------------------------------*/
        GPIO_LED(LED_ON);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_LED(LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreate(MC3416_Task,"MC3416Task",1024*4,NULL,5,NULL);
}
