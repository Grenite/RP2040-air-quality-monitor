#include <gfx.h>

#if GFX_USE_GINPUT && GINPUT_NEED_MOUSE
#define GMOUSE_DRIVER_VMT		GMOUSEVMT_TSC2007
#include <ginput_driver_mouse.h>
#include "gmouse_lld_TSC2007_board.h"
#include "I2C-Mutex.h"
// #include <stdio.h>

static GFXINLINE gU16 write_command (GMouse* m, gU8 reg, adafruit_tsc2007_function func,
                                   adafruit_tsc2007_power pwr,
                                   adafruit_tsc2007_resolution res)
{
    gU8 cmd = (gU8)func << 4;
    cmd |= (gU8)pwr << 2;
    cmd |= (gU8)res << 1;
    // printf("writing command\n");
    write_reg(m, reg, cmd, true);
    gU16 reply = read_word(m, reg, false);
    // printf("reading command\n");
  return reply;
}

static gBool MouseInit(GMouse* m, unsigned driverinstance) {
    ((TSC2007Mouse*)m)->mutex = &i2c_mutex;
    aquire_bus(m);
    // printf("Mouse Init\n");
    gBool result = init_board(m, driverinstance);
    if (result)
    {
        write_command(m, TSC2007_I2CADDR_DEFAULT, SETUP_COMMAND, POWERDOWN_IRQON, ADC_12BIT);
    }
    release_bus(m);
    return result;
}

static gBool read_xyz(GMouse* m, GMouseReading* pdr) {
    aquire_bus(m);
    gU16 x, y, z1, z2;
    // printf("Read xyz\n");
    z1 = write_command(m, TSC2007_I2CADDR_DEFAULT, MEASURE_Z1, ADON_IRQOFF, ADC_12BIT);
    z2 = write_command(m, TSC2007_I2CADDR_DEFAULT, MEASURE_Z2, ADON_IRQOFF, ADC_12BIT);
  // take two measurements since there can be a 'flicker' on pen up
    x = write_command(m, TSC2007_I2CADDR_DEFAULT, MEASURE_X, ADON_IRQOFF, ADC_12BIT);
    y = write_command(m, TSC2007_I2CADDR_DEFAULT, MEASURE_Y, ADON_IRQOFF, ADC_12BIT);

    pdr->buttons = 0;
    pdr->x = (gCoord) x;
    pdr->y = (gCoord) y;
    pdr->z = (gCoord) z1;
    // if (z1 > 0x0)
    // {
    //     printf("x: %u, y: %u, z1: %u, z2: %u\n", x, y, z1, z2);
    // }
    write_command(m, TSC2007_I2CADDR_DEFAULT, MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT);
    release_bus(m);
}

const GMouseVMT const GMOUSE_DRIVER_VMT[1] = {{
	{
		GDRIVER_TYPE_TOUCH,
        GMOUSE_VFLG_TOUCH | GMOUSE_VFLG_ONLY_DOWN | GMOUSE_VFLG_DEFAULTFINGER | GMOUSE_VFLG_CALIBRATE,
		sizeof(TSC2007Mouse) + GMOUSE_TSC2007_BOARD_DATA_SIZE,
		_gmouseInitDriver,
		_gmousePostInitDriver,
		_gmouseDeInitDriver
	},
	4095,				// z_max	- 0 indicates full touch
	0,			// z_min
	50,			// z_touchon
	25,			// z_touchoff
	{				// pen_jitter
		GMOUSE_TSC2007_PEN_CALIBRATE_ERROR,		// calibrate
		GMOUSE_TSC2007_PEN_CLICK_ERROR,			// click
		GMOUSE_TSC2007_PEN_MOVE_ERROR				// move
	},
	{				// finger_jitter
		GMOUSE_TSC2007_FINGER_CALIBRATE_ERROR,		// calibrate
		GMOUSE_TSC2007_FINGER_CLICK_ERROR,			// click
		GMOUSE_TSC2007_FINGER_MOVE_ERROR			// move
	},
	MouseInit, 		// init
	0,				// deinit
	read_xyz,		// get
	0,				// calsave
	0				// calload
}};

#endif