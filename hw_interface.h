#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include <stdint.h>

/*********************
 *      Delays
 *********************/

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/*********************
 *      GPIO
 *********************/

void pin_cmd_data_set(int mode);
void pin_rst_set(int mode);
void spi_cs_set(int mode);

/*********************
 *      SPI
 *********************/

void spi_wr(uint8_t data);
void spi_wr_mem(uint8_t* data, uint16_t len);

/*********************
 *      I2C
 *********************/

void i2c_start();
void i2c_stop();
void i2c_restart();
void i2c_wr(uint8_t data);
uint8_t i2c_rd();

#endif