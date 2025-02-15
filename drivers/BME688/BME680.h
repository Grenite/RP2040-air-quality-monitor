#pragma once
#include "bme68x.h"
#include "hardware/i2c.h"
#include <FreeRTOS.h>
#include <semphr.h>

#define BME68X_DEFAULT_ADDRESS (0x77)    ///< The default I2C address
#define BME68X_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

#define BME680_OS_16X BME68X_OS_16X   ///< Alias for BME680 existing examples
#define BME680_OS_8X BME68X_OS_8X     ///< Alias for BME680 existing examples
#define BME680_OS_4X BME68X_OS_4X     ///< Alias for BME680 existing examples
#define BME680_OS_2X BME68X_OS_2X     ///< Alias for BME680 existing examples
#define BME680_OS_1X BME68X_OS_1X     ///< Alias for BME680 existing examples
#define BME680_OS_NONE BME68X_OS_NONE ///< Alias for BME680 existing examples

#define BME680_FILTER_SIZE_127                                                 \
  BME68X_FILTER_SIZE_127 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_63                                                  \
  BME68X_FILTER_SIZE_63 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_31                                                  \
  BME68X_FILTER_SIZE_31 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_15                                                  \
  BME68X_FILTER_SIZE_15 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_7                                                   \
  BME68X_FILTER_SIZE_7 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_3                                                   \
  BME68X_FILTER_SIZE_3 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_1                                                   \
  BME68X_FILTER_SIZE_1 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_0                                                   \
  BME68X_FILTER_OFF ///< Alias for BME680 existing examples

typedef struct bme680_t {
    i2c_inst_t* i2c_inst;
    SemaphoreHandle_t* mutex;
    int32_t sensorID;
    uint32_t meas_start;
    uint16_t meas_period;
    float temperature_raw;
    uint32_t pressure;
    float humidity_raw;
    uint32_t gas_resistance_raw;
    struct bme68x_dev gas_sensor;
    struct bme68x_conf gas_conf;
    struct bme68x_heatr_conf gas_heatr_conf;
} bme680_t;

bme680_t *bme680_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl);
bool bme680_begin(bme680_t* obj);
void bme680_deinit(bme680_t* obj);

bool bme680_performReading(bme680_t* obj);
bool bme680_endReading(bme680_t* obj);
uint32_t bme680_beginReading(bme680_t* obj);

bool bme680_setIIRFilterSize(bme680_t* obj, uint8_t filtersize);
bool bme680_setODR(bme680_t* obj, uint8_t odr);
bool bme680_setHumidityOversampling(bme680_t* obj, uint8_t oversample);
bool bme680_setPressureOversampling(bme680_t* obj, uint8_t oversample);
bool bme680_setTemperatureOversampling(bme680_t* obj, uint8_t oversample);
bool bme680_setGasHeater(bme680_t* obj, uint16_t heaterTemp, uint16_t heaterTime);

int bme680_remainingReadingMillis(bme680_t* obj);

float bme680_readTemperature(bme680_t* obj);
float bme680_readPressure(bme680_t* obj);
float bme680_readHumidity(bme680_t* obj);
uint32_t bme680_readGas(bme680_t* obj);
float bme680_readAltitude(bme680_t* obj, float seaLevel);
