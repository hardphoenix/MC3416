# MC3416 Accelerometer Library

# If you are a user of IC (Accelerometer) MC3416 library You need to do the following steps to port this library with your code:
# This library is ported for [ESP32-IDF](https://github.com/espressif/esp-idf) , [STM32](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html) Mcrocontroller.

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
```

- You Can Download DataSheet MC3416 From This Link: 
-[MC3416](https://www.memsic.com/Public/Uploads/uploadfile/files/20220522/MC3416Datasheet(APS-045-0020v2.2).pdf)


-If you use the RTOS operating system, you must change the value of the following macro at the beginning of the MC3416.h file:
```C
#define USE_RTOS    1
```
