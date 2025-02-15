#include "TSC2007.h"
#include <ginput.h>
#include <ginput_driver_mouse.h>
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include <semphr.h>
typedef struct TSC2007Mouse {
    struct GMouse;
    SemaphoreHandle_t* mutex;
} TSC2007Mouse;

// #include <stdio.h>
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA PICO_DEFAULT_I2C_SDA_PIN 
#define I2C_SCL PICO_DEFAULT_I2C_SCL_PIN


static GFXINLINE gBool init_board(GMouse* m, unsigned driverinstance) {
    // i2c_init(I2C_PORT, TSC2007_I2C_BAUDRATE);
    
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
    return gTrue;
}


static GFXINLINE void aquire_bus(GMouse* m) {
    xSemaphoreTake(*((TSC2007Mouse*)m)->mutex, portMAX_DELAY);
}

static GFXINLINE void release_bus(GMouse* m) {
    xSemaphoreGive(*((TSC2007Mouse*)m)->mutex);
}

static GFXINLINE void write_reg(GMouse* m, gU8 reg, gU8 val, bool nostop) {
    const int written = i2c_write_blocking(I2C_PORT, (uint8_t)reg, (uint8_t*)&val, sizeof(val), nostop);
    // printf("Bytes Written: %d", written);
}

static GFXINLINE gU8 read_byte(GMouse* m, gU8 reg, bool nostop) {
    gU8 result;
    i2c_read_blocking(I2C_PORT, reg, (uint8_t*)&result, sizeof(result), nostop);
    return result;
}

static GFXINLINE gU16 read_word(GMouse* m, gU8 reg, bool nostop) {
    gU16 result;
    uint8_t recv[2];
    i2c_read_blocking(I2C_PORT, reg, (uint8_t*)recv, sizeof(recv), nostop);
    //12-bit data read - refer to datasheet for diagram
    result = (gU16)recv[0] << 4 | recv[1] >> 4;
    return result;
}