#include "lv_drv_conf.h"

/*********************
 *      Defines
 *********************/

#define SPI_DEV_CS      8
#define SPI_DATA_CMD    7
#define DISP_RST        5

#define SPI_DEV         "/dev/spidev0.0"
#define SPI_FREQUENCY   75000000

#define I2C_OPENED      1
#define I2C_CLOSED      0
#define I2C_DEV_FD      "/dev/i2c-1"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pigpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <assert.h>

/*********************
 *      Static
 *********************/

static uint8_t setup = 0;

struct i2c_device {
    int fd;
    uint16_t addr;
    uint8_t status;
};

static struct i2c_device ft5406ee8_i2c_dev = {
    .fd = -1,
    #if USE_FT5406EE8
    .addr = FT5406EE8_I2C_ADR,
    #else
    .addr = 0x00,
    #endif
    .status = I2C_CLOSED
};

static void gpio_setup(){
    if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return 1;
   }
   gpioSetMode(SPI_DATA_CMD, PI_OUTPUT);
   gpioSetMode(DISP_RST, PI_OUTPUT);
}

static void spi_init(int* fd){
    *fd = open(SPI_DEV, O_RDWR);

    /*uint8_t mode = SPI_MODE_1;
    if(!ioctl(fd, SPI_IOC_WR_MODE, SPI_MODE_1)){
        perror("Error setting SPI mode");
        exit(1);
    }

    uint32_t speed = 500000;
    if(!ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)){
        perror("Error settings max SPI speed");
        exit(1);
    }*/
}

/*********************
 *      Delays
 *********************/

void delay_us(uint32_t us){
    usleep(us);
}

void delay_ms(uint32_t ms){
    usleep(ms * 1000);
}

/*********************
 *      GPIO
 *********************/

void pin_cmd_data_set(int mode){
    if(!setup){
        gpio_setup();
    }
    gpioWrite(SPI_DATA_CMD, mode);
}

void pin_rst_set(int mode){
    if(!setup){
        gpio_setup();
    }
    gpioWrite(DISP_RST, mode);
}

void spi_cs_set(int mode){
    if(!setup){
        gpio_setup();
    }
    digitalWrite(SPI_DEV_CS, mode);
}

/*********************
 *      SPI
 *********************/

void spi_wr(uint8_t data){
    spi_cs_set(0);
    int fd;
    spi_init(&fd);
    uint8_t mode;
    ioctl(fd, SPI_IOC_RD_MODE, &mode);
    struct spi_ioc_transfer tx;
    memset(&tx, 0, sizeof(tx));
    uint8_t rx[1];
    tx.tx_buf = (unsigned long) &data;
    //tx.tx_nbits = 1;
    tx.rx_buf = (unsigned long) rx;
    //tx.rx_nbits = 3;
    tx.len = 1;
    tx.speed_hz = SPI_FREQUENCY;
    tx.cs_change = 0;
    //tx.bits_per_word = 8;
    if(ioctl(fd, SPI_IOC_MESSAGE(1), &tx) < 0){
        perror("Error writing to /dev/spidev0.0");
        exit(1);
    }
    close(fd);
    spi_cs_set(1);
}

void spi_wr_mem(uint8_t* data, uint16_t len){
    spi_cs_set(0);
    int fd;
    spi_init(&fd);
    struct spi_ioc_transfer tx;
    memset(&tx, 0, sizeof(tx));
    //uint8_t tx_buf[len];
    tx.tx_buf = (unsigned long) data;
    tx.len = len;
    tx.speed_hz = SPI_FREQUENCY;
    tx.cs_change = 0;
    if(ioctl(fd, SPI_IOC_MESSAGE(1), &tx) < 0){
        perror("Error writing multiple to /dev/spidev0.0");
        exit(1);
    }
    close(fd);
    spi_cs_set(1);
}

/*********************
 *      I2C
 *********************/

static void i2c_init(){
    if(ft5406ee8_i2c_dev.status != I2C_CLOSED){
        printf("Error opening /dev/i2c-1: i2c already started\n");
        exit(1);
    }
    ft5406ee8_i2c_dev.fd = open(I2C_DEV_FD, O_RDWR);
    if(ft5406ee8_i2c_dev.fd < 0){
        perror("Error opening /dev/i2c-1");
        exit(1);
    }
    if(ioctl(ft5406ee8_i2c_dev.fd, I2C_SLAVE, ft5406ee8_i2c_dev.addr) < 0){
        perror("Error writing to /dev/i2c-1");
        exit(1);
    }
    ft5406ee8_i2c_dev.status = I2C_OPENED;
}

static void i2c_close(){
    if(ft5406ee8_i2c_dev.status != I2C_OPENED){
        printf("Error closing /dev/i2c-1: i2c not started\n");
        exit(1);
    }
    close(ft5406ee8_i2c_dev.fd);
    ft5406ee8_i2c_dev.status = I2C_CLOSED;
}

void i2c_start(){
    return;
}

void i2c_stop(){
    return;
}

void i2c_restart(){
    return;
}

void i2c_wr(uint8_t data){
    i2c_init();
    if(!write(ft5406ee8_i2c_dev.fd, &data, 1)){
        perror("Error writing to /dev/i2c-1");
        exit(1);
    }
    i2c_close();
}

uint8_t i2c_rd(){
    i2c_init();
    uint8_t byte;
    if(read(ft5406ee8_i2c_dev.fd, &byte, 1) != 1){
        perror("Error reading from /dev/i2c-1");
        exit(1);
    }
    i2c_close();
    return byte;
}


