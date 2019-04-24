#ifndef _J_I2C_h_
#define _J_I2C_h_

#include "driver/gpio.h"
/**************************************************************************/
//软件I2C驱动---RTP5901 & ES8374 共用I2C
#define I2C_SCL_GPIO  19
#define I2C_SDA_GPIO  18

#define  I2C_SDA_OUT()     gpio_set_direction(I2C_SDA_GPIO,GPIO_MODE_OUTPUT_OD) 
#define  I2C_SDA_IN()       gpio_set_direction(I2C_SDA_GPIO,GPIO_MODE_INPUT)
#define  I2C_SCL_OUT()     gpio_set_direction(I2C_SCL_GPIO, GPIO_MODE_OUTPUT_OD);

#define  I2C_SDA_OUT_H    gpio_set_level(I2C_SDA_GPIO,1) 
#define  I2C_SDA_OUT_L    gpio_set_level(I2C_SDA_GPIO,0)

#define  I2C_SCL_OUT_H    gpio_set_level(I2C_SCL_GPIO,1)
#define  I2C_SCL_OUT_L    gpio_set_level(I2C_SCL_GPIO,0)

#define  I2C_READ_SDA()   gpio_get_level(I2C_SDA_GPIO) 
/**************************************************************************/
void I2C_Read_Bytes(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *str,uint8_t length);
void I2C_Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t I2C_Single_Read(uint8_t SlaveAddress,uint8_t REG_Address);
void I2C_continuous_write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *pw,uint8_t quantity);	

void delay_us(unsigned  int num);


#endif
