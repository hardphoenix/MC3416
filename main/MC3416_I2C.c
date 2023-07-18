/**
 * @file MC3416_I2C.c
 * @author MH.Taheri [HPX] (etatosel@gmail.com)
 * @version 1
 * 
 * 
 * @date 2023-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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


void gpio_Task(void);
/// @brief LED Status ///////////////////////////////
typedef enum
{
    LED_ON=1,
    LED_OFF=0,
}LED_STATE_E;

#define LED_PIN             ((1ULL<<GPIO_NUM_2))
#define INTR_INPUT_PIN      ((1ULL<<GPIO_NUM_4))

/*-----------------------------------*/
static QueueHandle_t gpio_in_queue = NULL;
/*-----------------------------------*/

static void IRAM_ATTR my_gpio_isr_handler(void *arg)
{
    uint32_t pin = (uint32_t) arg;
    xQueueSendFromISR(gpio_in_queue,&pin,NULL); 
}

void GPIO_INIT(void)
{
    gpio_config_t gpio_led,gpio_intr;

    gpio_led.intr_type= GPIO_INTR_DISABLE;
    gpio_led.mode= GPIO_MODE_OUTPUT;
    gpio_led.pin_bit_mask = LED_PIN;
    gpio_led.pull_down_en =0;
    gpio_led.pull_up_en=0;
    gpio_set_direction(GPIO_NUM_2,GPIO_MODE_OUTPUT);
    gpio_config(&gpio_led);
    xTaskCreate(gpio_Task,"ledTask",1024*1,NULL,3,NULL);     /*Make Task LED Blink*/

    gpio_intr.intr_type=GPIO_INTR_NEGEDGE;
    gpio_intr.mode=GPIO_MODE_INPUT;
    gpio_intr.pin_bit_mask=INTR_INPUT_PIN;
    gpio_intr.pull_up_en=0;
    gpio_intr.pull_up_en=1;
    gpio_config(&gpio_intr);

    gpio_set_intr_type(GPIO_NUM_4,GPIO_INTR_NEGEDGE);
    gpio_in_queue = xQueueCreate(20,sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_4,my_gpio_isr_handler,(void*)GPIO_NUM_4);
}

void GPIO_LED(LED_STATE_E LED_STAT)
{
    gpio_set_level(GPIO_NUM_2,(uint32_t)LED_STAT);
}

void gpio_Task(void)
{

    while (1)
    {
        GPIO_LED(LED_ON);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_LED(LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
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
    static int32_t xi,yi,zi;
    MC34xx_Interrupt_t MC34xx_intn_list;
    uint32_t pin_mask=0;
    while (1)
    {
        /*-----------------------------------*/
        MC34xx_Get_XYZ_Float(&x_axis,&y_axis,&z_axis);
        MC34xx_Get_XYZ_RowData(&xi,&yi,&zi);
        
        // MC34xx_GetStatus_Tilt(my_mc34xx_motion_cb);
        printf("\n----------------\n");
        /*-----------------------------------*/
        //Software Mode 
        //Poolin Mode Check Register ISR MC34xx
        MC34xx_intn_list = MC34xx_CheckSoft_Interrupt();    /*Check ISR Register*/
        switch ((uint8_t)MC34xx_intn_list)
        {
        case MC_INTN_No:
            // printf("\nNo Any INTN");
            break;
        case MC_INTN_Tilt:
            printf("\n--->Tilt INT Detect!");
            break;
        case MC_INTN_Flip:
            printf("\n--->Flip INT Detect!");
            break;
        case MC_INTN_AnyMotion:
            printf("\n--->AnyMotion INT Detect!");
            break;
        case MC_INTN_Shake:
            printf("\n--->Shake INT Detect!");
            break;
        case MC_INTN_Tilt_35:
            printf("\n--->Tilt_35 INT Detect!");
            break;
        case MC_INTN_ACQ:
            printf("\n--->ACQ INT Detect!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        /*-----------------------------------*/
        //Hardware Mode
        //Check ISR From MC34xx With INTN Hardware Pin After Check Register 
        // MC34xx_CheckHard_Interrupt();
        // if(xQueueReceive(gpio_in_queue, &pin_mask,100 /*portMAX_DELAY*/))
        // {
        //     printf("\n->ISR From Pin=%"PRIu32"",pin_mask);
        //     MC34xx_intn_list = MC34xx_CheckSoft_Interrupt();    /*Check ISR Register*/
        //     switch ((uint8_t)MC34xx_intn_list)
        //     {
        //     case MC_INTN_No:
        //         printf("\nNo Any INTN");
        //         break;
        //     case MC_INTN_Tilt:
        //         printf("\n--->Tilt INT Readed!");
        //         break;
        //     case MC_INTN_Flip:
        //         printf("\n--->Flip INT Readed!");
        //         break;
        //     case MC_INTN_AnyMotion:
        //         printf("\n--->AnyMotion INT Readed!");
        //         break;
        //     case MC_INTN_Shake:
        //         printf("\n--->Shake INT Readed!");
        //         break;
        //     case MC_INTN_Tilt_35:
        //         printf("\n--->Tilt_35 INT Readed!");
        //         break;
        //     case MC_INTN_ACQ:
        //         printf("\n--->ACQ INT Readed!");
        //         break;
        //     }
        // }
        /*-----------------------------------*/
    }
}


void app_main(void)
{
    xTaskCreate(MC3416_Task,"MC3416Task",1024*2,NULL,5,NULL);
}
