/**
 * @file I2C_Drive.h
 * @author MH.Taheri [HPX] (etatosel@gmail.com)
 * @brief I2c Drive For ESP32 And MC60
 * @version 1
 * @date 2023-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _I2C_DRIVE_H
#define _I2C_DRIVE_H
#include <stdint.h>


//////////////////////////////////
#define USE_RTOS        1

#define ESP32_IDF
// #define MC60_OPENCPU
// #define STM32
//////////////////////////////////

#define ACK_CHECK_EN        (0x01)
#define ACK_CHECK_DISABLE   (0x00)
#define NACK_EN             (0x01)

#ifdef ESP32_IDF
#define I2C_MASTER_TX_BUF       0
#define I2C_MASTER_RX_BUF       0
#define I2C_0   0
#define I2C_1   1
#endif


typedef enum
{
    ex_i2c_ok=0,
    ex_i2c_error,
    ex_i2c_speed_error,
    ex_i2c_init_error,
    ex_i2c_write_error,
}ex_i2c_t;


static ex_i2c_t I2C_Drive_Init(void *i2c_num,uint32_t i2c_speed);
static ex_i2c_t I2C_Write_Byte(void *i2c_num, uint8_t chip_address, uint8_t reg_address, uint8_t data);
static ex_i2c_t I2C_Read_Byte(void *i2c_num, uint8_t chip_address, uint8_t reg_address, uint8_t *out_date);

#endif
