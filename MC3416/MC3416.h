#ifndef _MC3416_H
/**
 * @file MC3416.h
 * @author MH.Taheri [HPX] (etatosel@gmail.com)
 * @link https://github.com/hardphoenix
 * @brief MC3416 MEMSIC Accelerometer IC Driver
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
/*MC3416_ADDR if vpp Connect To GND 0x4C And If Connect To VDD Address is 0x6C*/
#define MC3416_ADDR      (0x4C)
#define REG_Dev_Status   (0x05)
#define REG_Intr_Ctrl    (0x06)
#define REG_Mode         (0x07)
#define REG_SampleRate   (0x08)
#define REG_MotionCtrl   (0x09)
#define REG_XOUT_EX_L  (0x0D)
#define REG_XOUT_EX_H  (0x0E)
#define REG_YOUT_EX_L  (0x0F)
#define REG_YOUT_EX_H  (0x10)
#define REG_ZOUT_EX_L  (0x11)
#define REG_ZOUT_EX_H  (0x12)
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
/*---------------------------------------*/
#define GET_LSB(H)      (uint8_t)((H >> 8) & 0x00ff)
#define GET_MSB(H)      (uint8_t)(H & 0x00ff)
/*---------------------------------------*/
#define Tilt_Flip_En            1
#define Tilt_Flip_Disable       0
#define Latch_En                1 
#define Latch_Disable           0
#define AnyMotion_En            1
#define AnyMotion_Disable       0 
#define Shake_En                1 
#define Shake_Disable           0
#define Tilt35_En               1 
#define Tilt35_Disable          0 
#define Z_AxisOrien_En          1 
#define Z_AxisOrien_Disable     0 
#define FilterMotion_En         1 
#define FilterMotion_Disable    0 
#define MotionReset_En          1 
#define MotionReset_Disable     0 
/*--------------------------------------*/
#define  TILT_INT_Active        1
#define  TILT_INT_Disable       0 
#define  FLIP_INT_Active        1
#define  FLIP_INT_Disable       0
#define  ANYM_INT_Active        1
#define  ANYM_INT_Disable       0
#define  SHAKE_INT_Active       1
#define  SHAKE_INT_Disable      0
#define  TILT_35_INT_Active     1
#define  TILT_35_INT_Disable    0
#define  AUTO_CLR_Active        1
#define  AUTO_CLR_Disable       0
#define  ACQ_INT_Active         1
#define  ACQ_INT_Disable        0
//////////////////////////////////////////
/// @brief G-Force Range Can Be Write in Register: REG_Range
typedef enum
{
    g_range_2g = 0x09,
    g_range_4g = 0x19,
    g_range_8g = 0x29,
    g_range_16g = 0x39,
    g_range_12g = 0x49
}Chip_Range_t;

/// @brief List All Sample rate 
typedef enum
{
    sample_rate_128_default = 0x00,
    sample_rate_256= 0x01,
    sample_rate_512= 0x02,
    sample_rate_1024=0x05,
}Chip_SampleRate_t;

/// @brief list of all mode 
typedef enum
{
    MC_Mode_Wake=0,
    MC_Mode_Standby=1
}MC34xx_Mode_t;

/// @brief list all Tilt-35 timer duration
typedef enum
{
    MC_Timer_CTRL_No=0,
    MC_Timer_CTRL_1_6s,
    MC_Timer_CTRL_1_8s,
    MC_Timer_CTRL_2_0s,
    MC_Timer_CTRL_2_2s,
    MC_Timer_CTRL_2_4s,
    MC_Timer_CTRL_2_6s,
    MC_Timer_CTRL_2_8s,
    MC_Timer_CTRL_3_0s,
}MC34xx_Timer_Ctrl_Tilt35_t;

/// @brief List of INTN Bit Mask
typedef enum
{
    MC_INTN_Tilt=0,
    MC_INTN_Flip,
    MC_INTN_AnyMotion,
    MC_INTN_Shake,
    MC_INTN_Tilt_35,
    MC_INTN_Auto_Clr,
    MC_INTN_ACQ,
    MC_INTN_No,
}MC34xx_Interrupt_t;

/// @brief List Flags of Tilt Algorithm
typedef enum
{
    MC_FLAG_Tilt=0,
    MC_Flag_Flip,
    MC_Flag_AnyMotion,
    MC_Flag_Shake,
    MC_Flag_Tilt35,
    MC_Flag_NewData,
}MC34xx_Flags_Tilt_t;

/// @brief List Motion CTRL Bit 
typedef enum
{
    MC_MOT_TiltFlip=0,
    MC_MOT_Latch=1,
    MC_MOT_AnyMotion=2,
    MC_MOT_Shake=3,
    MC_MOT_Tilt35=4,
    MC_MOT_Z_AxisOrient=5,
    MC_MOT_FilterMotionData=6,
    MC_MOT_MotionReset=7,
}MC34xx_MotionCtrl_t;

/// @brief List All Return Enum
typedef enum
{
    MC_OK=0,
    MC_HW_Error,
    MC_ADDR_Error,
    MC_Init_Error,
    MC_Wr_Error,
    MC_Rd_Error,
    MC_Rd_NoEqual,
    MC_ChipID_Error,
    MC_Timer_Set_Error,
}EX_Error;


typedef struct __attribute__((__packed__))
{
    uint8_t Device_Status;
    uint8_t MC_ChipID;
    uint16_t Tilt_Flip_Thrshold;
    uint8_t Tilt_Flip_Debounce;
    uint16_t AnyMotion_Threshold;
    uint8_t AnyMotion_Debounce;
    uint16_t Shake_Threshold;
    uint16_t P2P_Duration;
    uint8_t MotionCtrl;
    uint8_t INTNCtrl;
    bool Check_NewDataBit;
    Chip_SampleRate_t sample_rate;
    Chip_Range_t g_range;
    MC34xx_Timer_Ctrl_Tilt35_t tilt35_time_ctrl;
}MC34xx_ChipParam_t;


EX_Error MC34xx_Init(MC34xx_ChipParam_t *ex_conf);
EX_Error MC34xx_Get_XYZ_Float(float *X, float *Y, float *Z);
EX_Error MC34xx_Get_XYZ_RowData(int32_t *X_Axis,int32_t *Y_Axis, int32_t *Z_Axis);
uint8_t MC34xx_MakeByte_MotionCtrl (bool TF_En,bool LT_En,bool AM_En,bool SHK_En,bool T35_En,bool Z_Axis_En,
                                    bool Filter_En,bool MotionReset);

uint8_t MC34xx_MakeByte_InterruptCtrl(bool TILT_INT_EN,bool FLIP_INT_EN,bool ANYM_INT_EN,bool SHAKE_INT_EN,
                                      bool TILT_35_INT_EN, bool AUTO_CLR_EN, bool ACQ_INT_EN);

MC34xx_Interrupt_t MC34xx_CheckSoft_Interrupt(void);
EX_Error MC34xx_CheckHard_Interrupt(void);
typedef void (* tilt_resp_callback)(MC34xx_Flags_Tilt_t MC_Flags);
EX_Error MC34xx_GetStatus_Tilt(tilt_resp_callback _titl_cb);
EX_Error MC34xx_Set_MotionBlock_Reset(bool En);
EX_Error MC34xx_SetMode(MC34xx_Mode_t mode);
EX_Error MC34xx_SetTimeDuration_Tilt35(MC34xx_Timer_Ctrl_Tilt35_t time_duration);

#endif