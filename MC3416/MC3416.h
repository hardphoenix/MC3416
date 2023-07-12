#ifndef _MC3416_H
/**
 * @file MC3416.h
 * @date
 * @author MH.Taheri [HPX] (etatosel@gmail.com)
 * @brief 
 * @version 1
 * @date 2023-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdint.h>
#include <stdbool.h>

/*If Use Rtos Turn 1 else Set 0*/
#define USE_RTOS    1

/**
 * @brief All Register Of MC3416
 * 
 */
//////////////////////////////////////////
#define MC3416_ADDR      (0x4C)
#define REG_Dev_Status   (0x05)
#define REG_Intr_Ctrl    (0x06)
#define REG_Mode         (0x07)
#define REG_SampleRate   (0x08)
#define REG_MotionCtrl   (0x09)
#define REG_XOUT_EX_LSB  (0x0D)
#define REG_XOUT_EX_MSB  (0x0E)
#define REG_YOUT_EX_LSB  (0x0F)
#define REG_YOUT_EX_MSB  (0x10)
#define REG_ZOUT_EX_LSB  (0x11)
#define REG_ZOUT_EX_MSB  (0x12)
#define REG_Status_2        (0x13)
#define REG_Intr_Stat_2     (0x14)
#define REG_Chip_Id         (0x18)
#define REG_Range           (0x20)
#define REG_X_OFFSET_LSB    (0x21)
#define REG_X_OFFSET_MSB    (0x22)
#define REG_Y_OFFSET_LSB    (0x23)
#define REG_Y_OFFSET_MSB    (0x24)
#define REG_Z_OFFSET_LSB    (0x25)
#define REG_Z_OFFSET_MSB    (0x26)
#define REG_X_Gain          (0x27)
#define REG_Y_Gain          (0x28)
#define REG_Z_Gain          (0x29)
#define REG_TF_Thresh_LSB   (0x40)
#define REG_TF_Thresh_MSB   (0x41)
#define REG_TF_Debounce     (0x42)
#define REG_AM_Thresh_LSB   (0x43)
#define REG_AM_Thresh_MSB   (0x44)
#define REG_AM_Debounce     (0x45)
#define REG_SHK_Thresh_LSB  (0x46)
#define REG_SHK_Thresh_MSB  (0x47)
#define REG_PK_P2P_DUR_Thresh_LSB   (0x48)
#define REG_PK_P2P_DUR_Thresh_MSB   (0x49)
#define REG_Timer_Ctrl      (0x4A)

#define GET_LSB(H)      (uint8_t)((H >> 8) & 0x00ff)
#define GET_MSB(H)      (uint8_t)(H & 0x00ff)
//////////////////////////////////////////
/// @brief G-Force Range Can Be Write in Register: REG_Range
typedef enum
{
    g_range_2g = 0x00,
    g_range_4g = 0x01,
    g_range_8g = 0x02,
    g_range_16g = 0x03,
    g_range_12g = 0x04
}Chip_Range_t;

typedef enum
{
    sample_rate_128_default = 0x00,
    sample_rate_256= 0x01,
    sample_rate_512= 0x02,
    sample_rate_1024=0x05,
}Chip_SampleRate_t;

typedef enum
{
    Unknown_Device = 0,
    MC3416_VPP2GND =1,
    MC3416_VPP2VDD =2,
}MCxx_chip_type_t;

typedef enum
{
    INTN_No_En=0,
    INTN_Tilt_En,
    INTN_Flip_En,
    INTN_AnyMotion_En,
    INTN_Shake_En,
    INTN_Tilt_35,
    INTN_Auto_Clr_En,
    INTN_ACQ_En,
}Interrupt_List_t;

typedef enum
{
    MC_OK=0,
    MC_HW_Error,
    MC_ADDR_Error,
    MC_Init_Error,
}EX_Error;


typedef struct 
{
    uint8_t Device_Status;
    uint8_t MC_ChipID;
    uint16_t Tilt_Flip_Thrshold;
    uint8_t Tilt_Flip_Debounce;
    uint16_t AnyMotion_Threshold; 
    uint8_t AnyMotion_Debounce;
    uint16_t Shake_Threshold;
    uint16_t P2P_Duration;
    MCxx_chip_type_t chip_type;
    Interrupt_List_t interrupt_list;
    Chip_SampleRate_t sample_rate;
    Chip_Range_t g_range;
}MC34xx_ChipParam_t;


EX_Error MC34xx_Init(MC34xx_ChipParam_t *ex_conf);
EX_Error MC34xx_UpdateStatus(uint8_t *DeviceStatus);
EX_Error MC34xx_Get_XYZ_Float(float *X, float *Y, float *Z);


#endif