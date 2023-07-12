#include "I2C_Drive.h"
#include "driver/i2c.h"

#if(USE_RTOS==1)
#include "freertos/FreeRTOS.h"
#endif


#if(USE_RTOS==1)
#define I2C_Delay(x)    vTaskDelay(x / portTICK_PERIOD_MS)
#else
#define I2C_Delay(x)    for(volatile int i=0;i<250; i++){}
#endif

#ifdef ESP32_IDF
#define I2C_NUMBER(Drive)   I2C_NUM_##Drive
#define I2C1_MASTER_SDA      21
#define I2C1_MASTER_SCL      22

#endif


ex_i2c_t I2C_Drive_Init(void *i2c_num,uint32_t i2c_speed)
{
    if(i2c_speed > 1000000) return ex_i2c_speed_error;  //Max Speed Of MC34xx 1 Mhz 

    #ifdef ESP32_IDF
    int drive_i2c =0;
    if((int)i2c_num == I2C_NUM_0)drive_i2c = I2C_NUMBER(0);
    else if((int)i2c_num == I2C_NUM_1)drive_i2c = I2C_NUMBER(1);

    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C1_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C1_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = i2c_speed;
    esp_err_t check_err = i2c_param_config((i2c_port_t)drive_i2c, &conf);
    if(check_err != ESP_OK)return check_err;

    check_err = i2c_driver_install(drive_i2c, conf.mode, I2C_MASTER_RX_BUF, I2C_MASTER_TX_BUF, 0);
    if(check_err != ESP_OK)return ex_i2c_initdrive_error;
    #endif


    return ex_i2c_ok;
}

ex_i2c_t I2C_Write_Byte(void *i2c_num, uint8_t chip_address, uint8_t reg_address, uint8_t data)
{
    #ifdef ESP32_IDF
    esp_err_t check = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    check = i2c_master_cmd_begin((i2c_port_t)i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    if(check != ESP_OK)return ex_i2c_write_error;
    i2c_cmd_link_delete(cmd);
    #endif
    I2C_Delay(1);
    return ex_i2c_ok;
}

ex_i2c_t I2C_Read_Byte(void *i2c_num, uint8_t chip_address, uint8_t reg_address, uint8_t *out_date)
{
    #ifdef ESP32_IDF
    esp_err_t check = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_address << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, out_date, NACK_EN);
    i2c_master_stop(cmd);
    check = i2c_master_cmd_begin((i2c_port_t)i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    #endif
    I2C_Delay(1);
    return ex_i2c_ok;
}
