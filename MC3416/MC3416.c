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

#define MC34_I2C_Drive      I2C_1
/*-----------------------------------*/
/**
 * @brief I2C Write Data Macro
 * @param Input Register Address and 1 Byte Data
 */
#define i2c_wr_byte(ChipAddress,RegAddress,Data)        I2C_Write_Byte(MC34_I2C_Drive,ChipAddress,RegAddress,Data)
/**
 * @brief I2C Read Data Macro 
 * @param OutData Is Pinter
 */
#define i2c_rd_byte(ChipAddress,RegAddress,OutData)     I2C_Read_Byte(MC34_I2C_Drive,ChipAddress,RegAddress,OutData)  
/*-----------------------------------*/
MC34xx_ChipParam_t *MC34xx_Config;

EX_Error MC34xx_Init(MC34xx_ChipParam_t *ex_conf)
{
    I2C_Drive_Init(MC34_I2C_Drive,400000);
    MC34xx_Config = ex_conf;
    
    /*Start Config */

    return MC_OK;
}

