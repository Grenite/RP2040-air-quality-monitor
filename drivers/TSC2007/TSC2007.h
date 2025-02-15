#define TSC2007_I2CADDR_DEFAULT 0x48 ///< TSC2007 default i2c address
#define TSC2007_I2C_BAUDRATE 100*1000
#define GMOUSE_TSC2007_PEN_CALIBRATE_ERROR		8
#define GMOUSE_TSC2007_PEN_CLICK_ERROR			6
#define GMOUSE_TSC2007_PEN_MOVE_ERROR			4
#define GMOUSE_TSC2007_FINGER_CALIBRATE_ERROR	14
#define GMOUSE_TSC2007_FINGER_CLICK_ERROR		18
#define GMOUSE_TSC2007_FINGER_MOVE_ERROR		14

// How much extra data to allocate at the end of the GMouse structure for the board's use
#define GMOUSE_TSC2007_BOARD_DATA_SIZE			0

/*!
 *    @brief  Different function commands
 */
typedef enum {
  MEASURE_TEMP0 = 0,
  MEASURE_AUX = 2,
  MEASURE_TEMP1 = 4,
  ACTIVATE_X = 8,
  ACTIVATE_Y = 9,
  ACTIVATE_YPLUS_X = 10,
  SETUP_COMMAND = 11,
  MEASURE_X = 12,
  MEASURE_Y = 13,
  MEASURE_Z1 = 14,
  MEASURE_Z2 = 15
} adafruit_tsc2007_function;

/*!
 *    @brief  Power and IRQ modes
 */
typedef enum {
  POWERDOWN_IRQON = 0,
  ADON_IRQOFF = 1,
  ADOFF_IRQON = 2,
} adafruit_tsc2007_power;

/*!
 *    @brief  ADC resolution
 */
typedef enum {
  ADC_12BIT = 0,
  ADC_8BIT = 1,
} adafruit_tsc2007_resolution;