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


EX_Error MC34xx_Check_WriteData(uint8_t LastReg, uint8_t LastWrData)
{
    uint8_t rd;
    if(i2c_rd_byte(MC3416_ADDR,LastReg,&rd) != ex_i2c_ok) return MC_Rd_Error;

    if(LastWrData != rd) return MC_Rd_NoEqual;

    debug("\r\nCheck REG=0x%02X=%0x02X",LastReg,rd);
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
    if(MC34xx_Get_CHipID() != MC_OK)debug("Get Chip ID With Error");

    if(MC34xx_Config->MC_ChipID != 0xA0) return MC_ChipID_Error;        //if chip id 0xA0 This is MC3416-P

    

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
    printf("X_Gain_float=%2.4f , Y_Gain_float=%2.4f, Z_Gain_float=%2.4f",MC34xx_private.X_Gain_float,
                            MC34xx_private.Y_Gain_float,MC34xx_private.Z_Gain_float);
    /*Start Config ------------------------------*/
    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x10); //stop sampling
    MC34xx_Check_WriteData(REG_Mode,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_Intr_Ctrl,0x1f);
    MC34xx_Check_WriteData(REG_Intr_Ctrl,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_MotionCtrl,0x3f);
    MC34xx_Check_WriteData(REG_MotionCtrl,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_Range,(uint8_t)MC34xx_Config->g_range);
    MC34xx_Check_WriteData(REG_Range,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_SampleRate,(uint8_t)MC34xx_Config->sample_rate);
    MC34xx_Check_WriteData(REG_SampleRate,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_X_Gain,MC34xx_private.X_Gain);
    MC34xx_Check_WriteData(REG_X_Gain,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_Y_Gain,MC34xx_private.Y_Gain);
    MC34xx_Check_WriteData(REG_Y_Gain,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_Z_Gain,MC34xx_private.Z_Gain);
    MC34xx_Check_WriteData(REG_Z_Gain,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_TF_Thresh_LSB,GET_LSB(MC34xx_Config->Tilt_Flip_Thrshold));
    MC34xx_Check_WriteData(REG_TF_Thresh_LSB,data);    //check
    i2c_wr_byte(MC3416_ADDR,REG_TF_Thresh_MSB,GET_MSB(MC34xx_Config->Tilt_Flip_Thrshold));
    MC34xx_Check_WriteData(REG_TF_Thresh_MSB,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_AM_Thresh_LSB,GET_LSB(MC34xx_Config->AnyMotion_Threshold));
    MC34xx_Check_WriteData(REG_AM_Thresh_LSB,data);    //check
    i2c_wr_byte(MC3416_ADDR,REG_AM_Thresh_MSB,GET_MSB(MC34xx_Config->AnyMotion_Threshold));
    MC34xx_Check_WriteData(REG_AM_Thresh_MSB,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_SHK_Thresh_LSB,GET_LSB(MC34xx_Config->Shake_Threshold));
    MC34xx_Check_WriteData(REG_SHK_Thresh_LSB,data);    //check
    i2c_wr_byte(MC3416_ADDR,REG_SHK_Thresh_MSB,GET_MSB(MC34xx_Config->Shake_Threshold));
    MC34xx_Check_WriteData(REG_SHK_Thresh_MSB,data);    //check

    i2c_wr_byte(MC3416_ADDR,REG_Mode,0x11); //start sampling
    MC34xx_Check_WriteData(REG_Mode,data);    //check

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
    if(i2c_rd_byte(MC3416_ADDR, REG_Chip_Id, &chipID) != MC_OK) return MC_Rd_Error;

    MC34xx_Config->MC_ChipID=chipID;
    return MC_OK;
}
