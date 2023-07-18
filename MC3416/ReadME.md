# MC3416 Accelerometer Library
- The MC3416 is a small form factor,integrated digital output 3-axis accelerometer with a feature set optimized for cell phones and consumer product motion sensing. Applications include user interface control, gaming motion input, electronic compass tilt compensation for cell phones, game controllers, remote controls and portable media products.

# Library features:
```bash
Get Accelerometer Xout Yout Zout Data.
The Accelerometer Resolution Range Configures ±2g , ±4g , ±8g , ±12g , ±16g
configure The Gain Value For 3 Axis.
Active Interrupt Pin For Motions Sense 

    Motion Algorithms: Tilt/Flip Tilt-35 
                       AnyMotion , Shake

Set Thershold Value For All Motion Algorithms.
Set Debounce Time For Motion Algorithm.
Configure the Time Control Register.
Configue The Output DataRate 128 To 1024 
```

- You Can Download DataSheet MC3416 From This Link: 
[MC3416](https://www.memsic.com/Public/Uploads/uploadfile/files/20220522/MC3416Datasheet(APS-045-0020v2.2).pdf)

- If you are a user of IC (Accelerometer) MC3416 library You need to do the following steps to port this library with your code:
- This library is ported for [ESP32-IDF](https://github.com/espressif/esp-idf) , [STM32](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html) Mcrocontroller.
- If you use the RTOS operating system, you must change the value of the following macro at the beginning of the MC3416.h file:

```C
#define USE_RTOS    1
```
- Set Device I2C Number  I2C_0 , I2C_1, ... in I2C_Drive.h:
```C
#define MC34_I2C_Drive      I2C_1
```
- uncomment Your CPU In I2C_Drive.h :
```c
#define ESP32_IDF
// #define MC60_OPENCPU
// #define STM32
```

- you Can use In Main.c Code:
```c
#include "MC3416.h"
//foo

int main()
{
    /*config and init mc34xx-----------*/
    MC34xx_ChipParam_t mc34xx_config = {0};
    mc34xx_config.g_range=g_range_2g;
    mc34xx_config.sample_rate=sample_rate_1024;
    mc34xx_config.X_Gain=0x0f;
    mc34xx_config.Y_Gain=0x0f;
    mc34xx_config.Z_Gain=0x0f;
    
    MC34xx_Init(&mc34xx_config);    //init mc3416 -> Disable any interrupt
    /*---------------------------------*/
    static float x_axis,y_axis,z_axis;  //3 float type for get axis

    while(1)
    {
        MC34xx_Get_XYZ_Float(&x_axis,&y_axis,&z_axis);
        MC34xx_intn_list = MC34xx_CheckSoft_Interrupt();    /*Check ISR Register*/
        switch ((uint8_t)MC34xx_intn_list)
        {
        case MC_INTN_No:
            printf("\nNo Any INTN");
            break;
        case MC_INTN_Tilt:
            printf("\n--->Tilt INT Readed!");
            break;
        case MC_INTN_Flip:
            printf("\n--->Flip INT Readed!");
            break;
        case MC_INTN_AnyMotion:
            printf("\n--->AnyMotion INT Readed!");
            break;
        case MC_INTN_Shake:
            printf("\n--->Shake INT Readed!");
            break;
        case MC_INTN_Tilt_35:
            printf("\n--->Tilt_35 INT Readed!");
            break;
        case MC_INTN_ACQ:
            printf("\n--->ACQ INT Readed!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));   //delay For Test 
        //foo
    }
}

```

## Author
- [MH.Taheri Github](https://github.com/hardphoenix) 
- [Telegram](https://t.me/mhtaheri_ir)
