/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#include "gfx.h"

#if GFX_USE_GDISP

#if defined(GDISP_SCREEN_HEIGHT) || defined(GDISP_SCREEN_HEIGHT)
	#if GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_DIRECT
		#warning "GDISP: This low level driver does not support setting a screen size. It is being ignored."
	#elif GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_MACRO
		COMPILER_WARNING("GDISP: This low level driver does not support setting a screen size. It is being ignored.")
	#endif
	#undef GDISP_SCREEN_WIDTH
	#undef GDISP_SCREEN_HEIGHT
#endif

#define GDISP_DRIVER_VMT			GDISPVMT_ILI9341
#include "gdisp_lld_config.h"
#include "../../../src/gdisp/gdisp_driver.h"

#include "board_ILI9341.h"
#include <stdio.h>
/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#ifndef GDISP_SCREEN_HEIGHT
	#define GDISP_SCREEN_HEIGHT		320
#endif
#ifndef GDISP_SCREEN_WIDTH
	#define GDISP_SCREEN_WIDTH		240
#endif
#ifndef GDISP_INITIAL_CONTRAST
	#define GDISP_INITIAL_CONTRAST	50
#endif
#ifndef GDISP_INITIAL_BACKLIGHT
	#define GDISP_INITIAL_BACKLIGHT	100
#endif

#include "ILI9341.h"

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Some common routines and macros
#define dummy_read(g)				{ volatile gU16 dummy; dummy = read_data(g); (void) dummy; }
#define write_reg(g, reg, data)		{ write_index(g, reg); write_data(g, data); }
#define write_data16(g, data)		{ write_data(g, data >> 8); write_data(g, (gU8)data); }
#define delay(us)					gfxSleepMicroseconds(us)
#define delayms(ms)					gfxSleepMilliseconds(ms)

typedef struct driver_data_t{
	gU16* display_buffer;
	uint32_t display_buffer_size;
	uint32_t display_buffer_used_size;
} driver_data_t;

static void set_viewport(GDisplay *g) {
	write_index(g, 0x2A);
	write_data(g, (g->p.x >> 8));
	write_data(g, (gU8) g->p.x);
	write_data(g, (g->p.x + g->p.cx - 1) >> 8);
	write_data(g, (gU8) (g->p.x + g->p.cx - 1));

	write_index(g, 0x2B);
	write_data(g, (g->p.y >> 8));
	write_data(g, (gU8) g->p.y);
	write_data(g, (g->p.y + g->p.cy - 1) >> 8);
	write_data(g, (gU8) (g->p.y + g->p.cy - 1));
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

LLDSPEC gBool gdisp_lld_init(GDisplay *g) {
	//Set private for now
	g->priv = 0; 

	// Initialise the board interface
	init_board(g);

	// Hardware reset
	setpin_reset(g, gTrue);
	gfxSleepMilliseconds(20);
	setpin_reset(g, gFalse);
	gfxSleepMilliseconds(20);

	// Get the bus for the following initialisation commands
	acquire_bus(g);

	write_index(g, 0x01); //software reset
	gfxSleepMilliseconds(5);
	write_index(g, 0x28);
	// display off
	//---------------------------------------------------------
	// magic?
	write_index(g, 0xcf);
	write_data(g, 0x00);
	write_data(g, 0x83);
	write_data(g, 0x30);

	write_index(g, 0xed);
	write_data(g, 0x64);
	write_data(g, 0x03);
	write_data(g, 0x12);
	write_data(g, 0x81);
	write_index(g, 0xe8);
	write_data(g, 0x85);
	write_data(g, 0x01);
	write_data(g, 0x79);
	write_index(g, 0xcb);
	write_data(g, 0x39);
	write_data(g, 0x2c);
	write_data(g, 0x00);
	write_data(g, 0x34);
	write_data(g, 0x02);
	write_index(g, 0xf7);
	write_data(g, 0x20);
	write_index(g, 0xea);
	write_data(g, 0x00);
	write_data(g, 0x00);
	//------------power control------------------------------
	write_index(g, 0xc0); //power control
	write_data(g, 0x26);
	write_index(g, 0xc1); //power control
	write_data(g, 0x11);
	//--------------VCOM
	write_index(g, 0xc5); //vcom control
	write_data(g, 0x35);//35
	write_data(g, 0x3e);//3E
	write_index(g, 0xc7); //vcom control
	write_data(g, 0xbe); // 0x94
	//------------memory access control------------------------
	write_index(g, 0x36);
	// memory access control
	write_data(g, 0x48); //0048 my,mx,mv,ml,BGR,mh,0.0
	write_index(g, 0x3a); // pixel format set
	write_data(g, 0x55);//16bit /pixel
	//----------------- frame rate------------------------------
	write_index(g, 0xb1);
	// frame rate
	write_data(g, 0x00);
	write_data(g, 0x1B); //70
	//----------------Gamma---------------------------------
	write_index(g, 0xf2); // 3Gamma Function Disable
	write_data(g, 0x08);
	write_index(g, 0x26);
	write_data(g, 0x01); // gamma set 4 gamma curve 01/02/04/08

	write_index(g, 0xE0); //positive gamma correction
	write_data(g, 0x1f);
	write_data(g, 0x1a);
	write_data(g, 0x18);
	write_data(g, 0x0a);
	write_data(g, 0x0f);
	write_data(g, 0x06);
	write_data(g, 0x45);
	write_data(g, 0x87);
	write_data(g, 0x32);
	write_data(g, 0x0a);
	write_data(g, 0x07);
	write_data(g, 0x02);
	write_data(g, 0x07);
	write_data(g, 0x05);
	write_data(g, 0x00);
	write_index(g, 0xE1); //negamma correction
	write_data(g, 0x00);
	write_data(g, 0x25);
	write_data(g, 0x27);
	write_data(g, 0x05);
	write_data(g, 0x10);
	write_data(g, 0x09);
	write_data(g, 0x3a);
	write_data(g, 0x78);
	write_data(g, 0x4d);
	write_data(g, 0x05);
	write_data(g, 0x18);
	write_data(g, 0x0d);
	write_data(g, 0x38);
	write_data(g, 0x3a);
	write_data(g, 0x1f);
	//--------------ddram ---------------------
	write_index(g, 0x2a);
	// column set
	// size = 239
	write_data(g, 0x00);
	write_data(g, 0x00);
	write_data(g, 0x00);
	write_data(g, 0xEF);
	write_index(g, 0x2b);
	// page address set
	// size = 319
	write_data(g, 0x00);
	write_data(g, 0x00);
	write_data(g, 0x01);
	write_data(g, 0x3F);
	// write_index(g, 0x34);
	//write_index(g, 0x35);
	// tearing effect off
	// tearing effect on
	// write_index(g, 0xb4); // display inversion
	// write_data(g, 0x00);
	write_index(g, 0xb7); //entry mode set
	write_data(g, 0x07);
	//-----------------display---------------------
	write_index(g, 0xb6);
	// display function control
	write_data(g, 0x0a);
	write_data(g, 0x82);
	write_data(g, 0x27);
	write_data(g, 0x00);
	write_index(g, 0x11); //sleep out
	gfxSleepMilliseconds(100);
	write_index(g, 0x29); // display on
	gfxSleepMilliseconds(100);

    // Finish Init
    post_init_board(g);

 	// Release the bus
	release_bus(g);
	
	/* Turn on the back-light */
	set_backlight(g, GDISP_INITIAL_BACKLIGHT);

	/* Initialise the GDISP structure */
	g->g.Width = GDISP_SCREEN_WIDTH;
	g->g.Height = GDISP_SCREEN_HEIGHT;
	g->g.Orientation = gOrientation0;
	g->g.Powermode = gPowerOn;
	g->g.Backlight = GDISP_INITIAL_BACKLIGHT;
	g->g.Contrast = GDISP_INITIAL_CONTRAST;
#if GDISP_HARDWARE_FILLS
	g->priv = (void*)gfxAlloc(sizeof(driver_data_t));
	((driver_data_t*)g->priv)->display_buffer_used_size = 0;
	((driver_data_t*)g->priv)->display_buffer_size = g->g.Width*g->g.Height;
	((driver_data_t*)g->priv)->display_buffer = (gU16*)gfxAlloc(((driver_data_t*)g->priv)->display_buffer_size*sizeof(gU16));
#endif
	return gTrue;
}

#if GDISP_HARDWARE_STREAM_WRITE
	LLDSPEC	void gdisp_lld_write_start(GDisplay *g) {
		acquire_bus(g);
		set_viewport(g);
		write_index(g, 0x2C);
	}
	LLDSPEC	void gdisp_lld_write_color(GDisplay *g) {
		write_data16(g, gdispColor2Native(g->p.color));
	}
	LLDSPEC	void gdisp_lld_write_stop(GDisplay *g) {
		release_bus(g);
	}
#endif

#if GDISP_HARDWARE_STREAM_READ
	LLDSPEC	void gdisp_lld_read_start(GDisplay *g) {
		acquire_bus(g);
		set_viewport(g);
		write_index(g, 0x2E);
		setreadmode(g);
		dummy_read(g);
	}
	LLDSPEC	gColor gdisp_lld_read_color(GDisplay *g) {
		gU16	data;

		data = read_data(g);
		return gdispNative2Color(data);
	}
	LLDSPEC	void gdisp_lld_read_stop(GDisplay *g) {
		setwritemode(g);
		release_bus(g);
	}
#endif

#if GDISP_NEED_CONTROL && GDISP_HARDWARE_CONTROL
	LLDSPEC void gdisp_lld_control(GDisplay *g) {
		switch(g->p.x) {
		case GDISP_CONTROL_POWER:
			if (g->g.Powermode == (gPowermode)g->p.ptr)
				return;
			switch((gPowermode)g->p.ptr) {
			case gPowerOff:
			case gPowerSleep:
			case gPowerDeepSleep:
				acquire_bus(g);
				write_reg(g, 0x0010, 0x0001);	/* enter sleep mode */
				release_bus(g);
				break;
			case gPowerOn:
				acquire_bus(g);
				write_reg(g, 0x0010, 0x0000);	/* leave sleep mode */
				release_bus(g);
				break;
			default:
				return;
			}
			g->g.Powermode = (gPowermode)g->p.ptr;
			return;

		case GDISP_CONTROL_ORIENTATION:
			if (g->g.Orientation == (gOrientation)g->p.ptr)
				return;
			switch((gOrientation)g->p.ptr) {
			case gOrientation0:
				acquire_bus(g);
				write_reg(g, 0x36, 0x48);	/* X and Y axes non-inverted */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_HEIGHT;
				g->g.Width = GDISP_SCREEN_WIDTH;
				break;
			case gOrientation90:
				acquire_bus(g);
				write_reg(g, 0x36, 0xE8);	/* Invert X and Y axes */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_WIDTH;
				g->g.Width = GDISP_SCREEN_HEIGHT;
				break;
			case gOrientation180:
				acquire_bus(g);
				write_reg(g, 0x36, 0x88);		/* X and Y axes non-inverted */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_HEIGHT;
				g->g.Width = GDISP_SCREEN_WIDTH;
				break;
			case gOrientation270:
				acquire_bus(g);
				write_reg(g, 0x36, 0x28);	/* Invert X and Y axes */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_WIDTH;
				g->g.Width = GDISP_SCREEN_HEIGHT;
				break;
			default:
				return;
			}
			g->g.Orientation = (gOrientation)g->p.ptr;
			return;

        case GDISP_CONTROL_BACKLIGHT:
            if ((unsigned)g->p.ptr > 100)
            	g->p.ptr = (void *)100;
            set_backlight(g, (unsigned)g->p.ptr);
            g->g.Backlight = (unsigned)g->p.ptr;
            return;

		//case GDISP_CONTROL_CONTRAST:
        default:
            return;
		}
	}
#endif

#if GDISP_HARDWARE_FILLS
LLDSPEC void gdisp_lld_fill_area(GDisplay *g) {
	const gU16 color = gdispColor2Native(g->p.color);
	// const gU8 byte[2] = {
	// 	(gU8)(color >> 8),
	// 	(gU8)color
	// };
	
	gdisp_lld_write_start(g);
	const uint32_t fillSize = g->p.cy*g->p.cx;
	driver_data_t* driver_data = (driver_data_t*)g->priv;
	driver_data->display_buffer_used_size = fillSize;
	for (uint32_t i = 0; i < fillSize; i++)
	{
		driver_data->display_buffer[i] = color;
	}
	// write_data_dma(g, driver_data->display_buffer, driver_data->display_buffer_used_size);
	write_data_block16(g, driver_data->display_buffer, driver_data->display_buffer_used_size);
	// write_data_block(g, driver_data->display_buffer, driver_data->display_buffer_used_size);
	// gU8 *buf  = (gU8*)gfxAlloc(sizeof(gU8) * (fillSize));


	// for (uint32_t i = 0; i < fillSize; i+=2)
	// {
	// 	// printf("%lu\n", i);
	// 	buf[i] = byte[0];
	// 	buf[i+1] = byte[1];
	// }

	
	// gfxFree(buf);


	gdisp_lld_write_stop(g);

}
#endif

#endif /* GFX_USE_GDISP */
