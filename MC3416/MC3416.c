/**
 * @file MC3416.c
 * @author MH.Taheri [HPX] (etatosel@gmail.com)
 * @brief 
 * @version 1
 * @date 2023-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "MC3416.h"
#include "driver/gpio.h"
#include "I2C_Drive.h"

/*Just Debug For Developer-------------------------------*/
#define dev_debug       (1)

#if dev_debug == 1
#define debug(...)      printf(__VA_ARGS__)
#else 
#define debug(...)      
#endif
/*Just Debug For Author Developer-------------------------------*/

#define MC34_I2C_Drive      I2C_1
/*-----------------------------------*/
// @brief I2C Write Data Macro 
// @param RegAddress Is Address Of Register
// @param Input Register Address and 1 Byte Data/
#define i2c_wr_byte(ChipAddress,RegAddress,Data)        I2C_Write_Byte((int)MC34_I2C_Drive,ChipAddress,RegAddress,Data)

// @brief I2C Read Data Macro 
// @param RegAddress Is Address Of Register
// @param OutData Is Pinter
#define i2c_rd_byte(ChipAddress,RegAddress,OutData)     I2C_Read_Byte((int)MC34_I2C_Drive,ChipAddress,RegAddress,OutData)  
/*-----------------------------------*/
typedef struct 
{   
    uint8_t X_Gain;
    uint8_t Y_Gain;
    uint8_t Z_Gain;
    float X_Gain_float;
    float Y_Gain_float;
    float Z_Gain_float;
    uint16_t X_Offset;
    uint16_t Y_Offset;
    uint16_t Z_Offset;
}MC34xx_Private_t;

MC34xx_Private_t    MC34xx_private;
/*----------------------------------*/
MC34xx_ChipParam_t *MC34xx_Config;
/*----------------------------------*/
EX_Error MC34xx_Get_CHipID(void);
/*----------------------------------*/

EX_Error MC34xx_Check_WriteData(uint8_t LastReg, uint8_t LastWrData)
{
    uint8_t rd;
    if(i2c_rd_byte(MC3416_ADDR,LastReg,&rd) != ex_i2c_ok) return MC_Rd_Error;

    if(LastWrData != rd) return MC_Rd_NoEqual;

    debug("\r\n<<--REG[0x%02X]->Write0x%02X , RdCheck=0x%02X",LastReg,LastWrData,rd);
    return MC_OK;
}

EX_Error MC34xx_Init(MC34xx_ChipParam_t *ex_conf)
{
    uint8_t data=0;
    ex_i2c_t err_i2c = ex_i2c_ok;
    err_i2c = I2C_Drive_Init((int)MC34_I2C_Drive,400);
    if(err_i2c != ex_i2c_ok)
    {
        debug("I2C_Drive_Init Error:0x%04X",err_i2c);
        return MC_Init_Error;
    }
    MC34xx_Config = ex_conf;
    /*----------------------------------------*/
    if(MC34xx_Get_CHipID() != MC_OK)debug("\nGet Chip ID With Error");

    if(MC34xx_Config->MC_ChipID != 0xA0) return MC_ChipID_Error;        //if chip id 0xA0 This is MC3416-P

    debug("\nChip ID =0x%02X",MC34xx_Config->MC_ChipID);

    /*----------------------------------------*/
    switch ((int)MC34xx_Config->g_range)
    {
    case g_range_2g:
        MC34xx_private.X_Gain = MC34xx_private.Y_Gain = MC34xx_private.Z_Gain=255;
        MC34xx_private.X_Gain_float = MC34xx_private.Y_Gain_float = MC34xx_private.Z_Gain_float = 1 / 255.0f;
        break;
    case g_range_4g:
        MC34xx_private.X_Gain = MC34xx_private.Y_Gain = MC34xx_private.Z_Gain=127;
        MC34xx_private.X_Gain_float = MC34xx_private.Y_Gain_float = MC34xx_private.Z_Gain_float = 1 / 127.0f;
        break;
    case g_range_8g:
        MC34xx_private.X_Gain = MC34xx_private.Y_Gain = MC34xx_private.Z_Gain=63;
        MC34xx_private.X_Gain_float = MC34xx_private.Y_Gain_float = MC34xx_private.Z_Gain_float = 1 / 63.0f;
        break;   
    case g_range_12g:
        MC34xx_private.X_Gain = MC34xx_private.Y_Gain = MC34xx_private.Z_Gain=42;
        MC34xx_private.X_Gain_float = MC34xx_private.Y_Gain_float = MC34xx_private.Z_Gain_float = 1 / 42.0f;
        break;
    case g_range_16g:
        MC34xx_private.X_Gain = MC34xx_private.Y_Gain = MC34xx_private.Z_Gain=31;
        MC34xx_private.X_Gain_float = MC34xx_private.Y_Gain_float = MC34xx_private.Z_Gain_float = 1 / 31.0f;
        break;
    }
    printf("\nX_Gain_float=%2.4f , Y_Gain_float=%2.4f, Z_Gain_float=%2.4f",MC34xx_private.X_Gain_float,
                            MC34xx_private.Y_Gain_float,MC34xx_private.Z_Gain_float);
    /*Start Config ------------------------------*/
    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x10); //stop sampling
    if(MC34xx_Check_WriteData(REG_Mode,0x10))return MC_Wr_Error;    //check

    // MC34xx_Config->interrupt_list
    i2c_wr_byte(MC3416_ADDR,REG_Intr_Ctrl,0x0f);
    if(MC34xx_Check_WriteData(REG_Intr_Ctrl,0x0f))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_MotionCtrl,MC34xx_Config->MotionCtrl);
    if(MC34xx_Check_WriteData(REG_MotionCtrl,MC34xx_Config->MotionCtrl))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_Range,(uint8_t)MC34xx_Config->g_range);
    if(MC34xx_Check_WriteData(REG_Range,(uint8_t)MC34xx_Config->g_range))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_SampleRate,(uint8_t)MC34xx_Config->sample_rate);
    if(MC34xx_Check_WriteData(REG_SampleRate,(uint8_t)MC34xx_Config->sample_rate))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_X_Gain,MC34xx_private.X_Gain);
    if(MC34xx_Check_WriteData(REG_X_Gain,MC34xx_private.X_Gain))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_Y_Gain,MC34xx_private.Y_Gain);
    if(MC34xx_Check_WriteData(REG_Y_Gain,MC34xx_private.Y_Gain))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_Z_Gain,MC34xx_private.Z_Gain);
    if(MC34xx_Check_WriteData(REG_Z_Gain,MC34xx_private.Z_Gain))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_TF_Thresh_LSB,GET_MSB(MC34xx_Config->Tilt_Flip_Thrshold));
    if(MC34xx_Check_WriteData(REG_TF_Thresh_LSB,GET_MSB(MC34xx_Config->Tilt_Flip_Thrshold)))return MC_Wr_Error;    //check
    i2c_wr_byte(MC3416_ADDR,REG_TF_Thresh_MSB,GET_LSB(MC34xx_Config->Tilt_Flip_Thrshold)& 0x7f);
    if(MC34xx_Check_WriteData(REG_TF_Thresh_MSB,GET_LSB(MC34xx_Config->Tilt_Flip_Thrshold)& 0x7f))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_AM_Thresh_LSB,GET_MSB(MC34xx_Config->AnyMotion_Threshold));
    if(MC34xx_Check_WriteData(REG_AM_Thresh_LSB,GET_MSB(MC34xx_Config->AnyMotion_Threshold)))return MC_Wr_Error;    //check
    i2c_wr_byte(MC3416_ADDR,REG_AM_Thresh_MSB,GET_LSB(MC34xx_Config->AnyMotion_Threshold)& 0x7f);
    if(MC34xx_Check_WriteData(REG_AM_Thresh_MSB,GET_LSB(MC34xx_Config->AnyMotion_Threshold)& 0x7f))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_SHK_Thresh_LSB,GET_MSB(MC34xx_Config->Shake_Threshold));
    if(MC34xx_Check_WriteData(REG_SHK_Thresh_LSB,GET_MSB(MC34xx_Config->Shake_Threshold)))return MC_Wr_Error;    //check
    i2c_wr_byte(MC3416_ADDR,REG_SHK_Thresh_MSB,GET_LSB(MC34xx_Config->Shake_Threshold));
    if(MC34xx_Check_WriteData(REG_SHK_Thresh_MSB,GET_LSB(MC34xx_Config->Shake_Threshold)))return MC_Wr_Error;    //check
    
    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x11); //start sampling
    if(MC34xx_Check_WriteData(REG_Mode,0x11))return MC_Wr_Error;    //check

    return MC_OK;
}


EX_Error MC34xx_Get_XYZ_Float(float *X, float *Y, float *Z)
{
    int data;
    uint8_t rd;
    i2c_rd_byte(MC3416_ADDR,REG_XOUT_EX_H,&rd);
    data =rd;
    i2c_rd_byte(MC3416_ADDR,REG_XOUT_EX_L,&rd);
    data = (uint16_t)((data << 8) | rd) & 0x3fff;     //0x3fff -> 14bit mode raw data
    printf("\nDX=%d",data);
    *X = data * MC34xx_private.X_Gain_float;

    i2c_rd_byte(MC3416_ADDR,REG_YOUT_EX_H,&rd);
    data =rd;
    i2c_rd_byte(MC3416_ADDR,REG_YOUT_EX_L,&rd);
    data = (uint16_t)(data << 8 | rd) & 0x3fff;
    printf(", DX=%d",data);
    *Y = data * MC34xx_private.Y_Gain_float;

    i2c_rd_byte(MC3416_ADDR,REG_ZOUT_EX_H,&rd);
    data =rd;
    i2c_rd_byte(MC3416_ADDR,REG_ZOUT_EX_L,&rd);
    data = (uint16_t)(data << 8 | rd) & 0x3fff;
    printf(", DX=%d",data);
    *Z = data * MC34xx_private.Z_Gain_float;

    return MC_OK;
}

EX_Error MC34xx_Get_CHipID(void)
{
    uint8_t chipID=0;
    if(i2c_rd_byte(MC3416_ADDR, REG_Chip_Id, &chipID) != ex_i2c_ok) return MC_Rd_Error;

    MC34xx_Config->MC_ChipID=chipID;
    return MC_OK;
}

/**
 * @brief Get Same Response Of All Motion Algorithms and Check New Data Converting In IC
 * 
 * @param _titl_cb
 * @return EX_Error 
 */
EX_Error MC34xx_GetStatus_Tilt(tilt_resp_callback _titl_cb)
{
    uint8_t rd=0;
    if(i2c_rd_byte(MC3416_ADDR,REG_Status_2,&rd) != ex_i2c_ok)return MC_Rd_Error;

    debug("\nStatusReg=0x%02X",rd);
    MC34xx_Set_MotionBlock_Reset(1);
    if((MC34xx_Config->MotionCtrl & 0x01))
        if((rd & 0x01)==1)_titl_cb(MC_FLAG_Tilt);

    if((MC34xx_Config->MotionCtrl & 0x01))
        if(((rd >> 1)& 0x01)==1)_titl_cb(MC_Flag_Flip);
        
    if(((MC34xx_Config->MotionCtrl >> MC_MOT_AnyMotion) & 0x01) == 1)
        if(((rd >> 2)& 0x01)==1)_titl_cb(MC_Flag_AnyMotion);
    
    if(((MC34xx_Config->MotionCtrl >> MC_MOT_Shake) & 0x01) == 1)
        if(((rd >> 3)& 0x01)==1)_titl_cb(MC_Flag_Shake);
    
    if(((MC34xx_Config->MotionCtrl >> MC_MOT_Tilt35) & 0x01) == 1)
        if(((rd >> 4)& 0x01)==1)_titl_cb(MC_Flag_Tilt35);
        
    if(MC34xx_Config->Check_NewDataBit)
        if(((rd >> 7)& 0x01)==1)_titl_cb(MC_Flag_NewData);
    
    return MC_OK;
}


EX_Error MC34xx_Set_MotionBlock_Reset(bool En)
{
    uint8_t bit_reset;
    if(En)bit_reset=0xff;
    else bit_reset=0x7f;

    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x10); //stop sampling
    if(MC34xx_Check_WriteData(REG_Mode,0x10) != MC_OK)return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_MotionCtrl,(MC34xx_Config->MotionCtrl & bit_reset));
    if(MC34xx_Check_WriteData(REG_MotionCtrl,(MC34xx_Config->MotionCtrl & bit_reset)))return MC_Wr_Error;    //check

    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x11); //start sampling
    if(MC34xx_Check_WriteData(REG_Mode,0x11))return MC_Wr_Error;    //check

    return MC_OK;
}

/**
 * @brief 
 * 
 * @param TF_En 
 * @param LT_En 
 * @param AM_En 
 * @param SHK_En 
 * @param T35_En 
 * @param Z_Axis_En 
 * @param Filter_En 
 * @param MotionReset 
 * @return uint8_t 
 */
uint8_t MC34xx_MakeByte_MotionCtrl (bool TF_En,bool LT_En,bool AM_En,bool SHK_En,bool T35_En,bool Z_Axis_En,
                                    bool Filter_En,bool MotionReset)
{
    uint8_t motion=0;
    if(TF_En) motion = motion | 1;
    if(LT_En)motion = motion | (1 << 1);
    if(AM_En)motion = motion | (1 << 2);
    if(SHK_En)motion = motion | (1 << 3);
    if(T35_En)motion = motion | (1 << 4);
    if(Z_Axis_En)motion = motion | (1 << 5);
    if(Filter_En)motion = motion | (1 << 6);
    if(MotionReset)motion = motion | (1 << 7);

    return motion;
}
                                    