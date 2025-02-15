/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include <gfx.h>

#define TFT_DC          10
#define PICO_DEFAULT_SPI_CSN_PIN 9
#define TFT_CS PICO_DEFAULT_SPI_CSN_PIN
#define TFT_MISO PICO_DEFAULT_SPI_RX_PIN
#define TFT_CLK PICO_DEFAULT_SPI_SCK_PIN
#define TFT_MOSI PICO_DEFAULT_SPI_TX_PIN

#include "board_ILI9341.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include <stdlib.h>
#include <src/gdisp/gdisp.h>
#include <src/gdisp/gdisp_driver.h>
static spi_inst_t *ili9341_spi = spi0;
static uint dma_channel_tx;
static dma_channel_config dma_cfg;
static uint64_t time;

static GFXINLINE void init_board(GDisplay *g) {
	// g->priv = NULL;
	g->board = NULL;
	time = 0;
    // SPI initialisation. This example will use SPI at 100MHz.
    spi_init(ili9341_spi, 1000 * 40000);
	spi_set_format(ili9341_spi, 8, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
    gpio_set_function(TFT_MISO, GPIO_FUNC_SPI);
    gpio_set_function(TFT_CS,   GPIO_FUNC_SIO);
    gpio_set_function(TFT_CLK,  GPIO_FUNC_SPI);
    gpio_set_function(TFT_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(TFT_DC, GPIO_FUNC_SIO);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(TFT_CS, GPIO_OUT);
    gpio_put(TFT_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

	gpio_set_dir(TFT_DC, GPIO_OUT);
	gpio_put(TFT_DC, 1);

	dma_channel_tx = dma_claim_unused_channel(true);
	dma_cfg = dma_channel_get_default_config(dma_channel_tx);
	channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&dma_cfg, spi_get_dreq(ili9341_spi, true));
	// channel_config_set_bswap(&dma_cfg, true);
}

static GFXINLINE void post_init_board(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setpin_reset(GDisplay *g, gBool state) {
	(void) g;
	(void) state;
	//reset pin not available for the tft being used - connected to button
}

static GFXINLINE void set_backlight(GDisplay *g, gU8 percent) {
	(void) g;
	(void) percent;
	//backlit pin not availabe for the tft being used - unless soldered in
}

static GFXINLINE void acquire_bus(GDisplay *g) {
	(void) g;
	gpio_put(TFT_CS, 0);
}

static GFXINLINE void release_bus(GDisplay *g) {
	(void) g;
	gpio_put(TFT_CS, 1);
}

//write command to command register
static GFXINLINE void write_index(GDisplay *g, gU16 index) {
	(void) g;
	const uint8_t data = (uint8_t)(index & 0x00FF);
	gpio_put(TFT_DC, 0);
	spi_set_format(ili9341_spi, 8, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
	spi_write_blocking(ili9341_spi, &data, sizeof(data));
}

static GFXINLINE void write_data(GDisplay *g, gU16 data) {
	(void) g;
	const uint8_t _data = (uint8_t)data;
	gpio_put(TFT_DC, 1);
	spi_set_format(ili9341_spi, 8, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
	spi_write_blocking(ili9341_spi, &_data, sizeof(_data));
}
static GFXINLINE void setreadmode(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setwritemode(GDisplay *g) {
	(void) g;
}

static GFXINLINE int read_data(GDisplay *g) {
	(void) g;
	gU16 data;
	spi_read_blocking(ili9341_spi, 0, (uint8_t *)&data, sizeof(data));
	return 0;
}

static GFXINLINE void write_data_dma(GDisplay *g, gU16* buf, uint32_t size)
{
	gpio_put(TFT_DC, 1);
	spi_set_format(ili9341_spi, 16, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
	// printf("%lu\n", size);
	dma_channel_configure(dma_channel_tx, &dma_cfg, 
						  &spi_get_hw(ili9341_spi)->dr, // write address
                          (void*)buf, // read address
                          size, // element count (each element is of size transfer_data_size)
                          true); 
	dma_channel_wait_for_finish_blocking(dma_channel_tx);
}

static GFXINLINE void write_data_block(GDisplay *g, gU8* data, size_t size) {
	(void) g;
	const uint8_t* _data = (uint8_t*)data;
	gpio_put(TFT_DC, 1);
	spi_set_format(ili9341_spi, 8, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
	spi_write_blocking(ili9341_spi, _data, size);
}

static GFXINLINE void write_data_block16(GDisplay *g, gU16* data, size_t size) {
	(void) g;
	const uint16_t* _data = (uint16_t*)data;
	gpio_put(TFT_DC, 1);
	spi_set_format(ili9341_spi, 16, SPI_CPOL_1, SPI_CPOL_1, SPI_MSB_FIRST);
	spi_write16_blocking(ili9341_spi, _data, size);
}


#endif /* _GDISP_LLD_BOARD_H */
