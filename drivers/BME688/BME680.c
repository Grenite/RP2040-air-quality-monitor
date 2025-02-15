#include "BME680.h"
#include <string.h>
#include <hardware/pio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/time.h"
#include <stdio.h>

// #define BME680_DEBUG

//reference: https://github.com/adafruit/Adafruit_BME680/blob/master/Adafruit_BME680.cpp#L445

/** Our hardware interface functions **/
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface);
static void delay_usec(uint32_t us, void *intf_ptr);

/** Value returned by remainingReadingMillis indicating no asynchronous
 * reading has been initiated by beginReading. **/
static const int reading_not_started = -1;
/** Value returned by remainingReadingMillis indicating asynchronous reading
 * is complete and calling endReading will not block. **/
static const int reading_complete = 0;

bool bme680_setIIRFilterSize(bme680_t* obj, uint8_t filtersize) 
{
    if (filtersize > BME68X_FILTER_SIZE_127)
        return false;
    obj->gas_conf.filter = filtersize;

    int8_t rslt = bme68x_set_conf(&(obj->gas_conf), &(obj->gas_sensor));

    return rslt == 0;
}

bool bme680_setODR(bme680_t* obj, uint8_t odr) 
{
  if (odr > BME68X_ODR_NONE)
    return false;

  obj->gas_conf.odr = odr;

  int8_t rslt = bme68x_set_conf(&(obj->gas_conf), &(obj->gas_sensor));
#ifdef BME680_DEBUG
#endif
  return rslt == 0;
}

bool bme680_setHumidityOversampling(bme680_t* obj, uint8_t oversample)
{
  if (oversample > BME68X_OS_16X)
    return false;

  obj->gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&(obj->gas_conf), &(obj->gas_sensor));
#ifdef BME680_DEBUG

#endif
  return rslt == 0;
}

bool bme680_setPressureOversampling(bme680_t* obj, uint8_t oversample)
{
  if (oversample > BME68X_OS_16X)
    return false;

  obj->gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&(obj->gas_conf), &(obj->gas_sensor));
#ifdef BME680_DEBUG
#endif
  return rslt == 0;
}

bool bme680_setTemperatureOversampling(bme680_t* obj, uint8_t oversample)
{
  if (oversample > BME68X_OS_16X)
    return false;

  obj->gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&(obj->gas_conf), &(obj->gas_sensor));
#ifdef BME680_DEBUG
#endif
  return rslt == 0;
}

bool bme680_setGasHeater(bme680_t* obj, uint16_t heaterTemp, uint16_t heaterTime)
{
  if ((heaterTemp == 0) || (heaterTime == 0)) {
    obj->gas_heatr_conf.enable = BME68X_DISABLE;
  } else {
    obj->gas_heatr_conf.enable = BME68X_ENABLE;
    obj->gas_heatr_conf.heatr_temp = heaterTemp;
    obj->gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt =
      bme68x_set_heatr_conf(BME68X_FORCED_MODE, &(obj->gas_heatr_conf), &(obj->gas_sensor));
#ifdef BME680_DEBUG
    printf("Set heater config result: %d\n", rslt);
#endif
  return rslt == 0;
}


bme680_t *bme680_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl)
{
    bme680_t *result = (bme680_t*)malloc(sizeof(bme680_t));
    memset(result, 0, sizeof(bme680_t));
    result->i2c_inst = i2c_inst;
    result->mutex = mutex;
    result->gas_sensor.chip_id = BME68X_CHIP_ID;
    result->gas_sensor.intf = BME68X_I2C_INTF;
    result->gas_sensor.intf_ptr = (void*)result;
    result->gas_sensor.read = &i2c_read;
    result->gas_sensor.write = &i2c_write;
    result->gas_sensor.amb_temp = 25;
    result->gas_sensor.delay_us = delay_usec;
    if (init_i2c)
    {
        i2c_init(i2c_inst, baudrate);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }
    return result;
}

bool bme680_begin(bme680_t* obj)
{
    int8_t rslt;
    rslt = bme68x_init(&(obj->gas_sensor));
    if (rslt != BME68X_OK)
    {
        printf("BME68x_init failed: %d\n", rslt);
        return false;
    }
    rslt = bme68x_soft_reset(&(obj->gas_sensor));
    if (rslt != BME68X_OK)
    {
        printf("BME68x_soft_reset failed: %d\n", rslt);
        return false;
    }
    // rslt = bme68x_selftest_check(&obj->gas_sensor);
    // if (rslt != BME68X_OK)
    // {
    //     printf("BME68x_selftest_check failed: %d\n", rslt);
    //     return false;
    // }
    bme680_setIIRFilterSize(obj, BME68X_FILTER_SIZE_3);
    bme680_setODR(obj, BME68X_ODR_NONE);
    bme680_setHumidityOversampling(obj, BME68X_OS_2X);
    bme680_setPressureOversampling(obj, BME68X_OS_2X);
    bme680_setTemperatureOversampling(obj, BME68X_OS_2X);
    bme680_setGasHeater(obj, 160, 50);
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &(obj->gas_sensor));

    return true;
}

void bme680_deinit(bme680_t* obj)
{
    free(obj);
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface)
{
    bme680_t* obj = (bme680_t*)interface;
    xSemaphoreTake(*(obj->mutex), portMAX_DELAY);
    int result = i2c_write_blocking(obj->i2c_inst, BME68X_DEFAULT_ADDRESS, &reg_addr, 1, true);
    // printf("%d bytes written\n", result);
    if (result > 0)
    {
       result = i2c_read_blocking(obj->i2c_inst,  BME68X_DEFAULT_ADDRESS, reg_data, (size_t)len, false);
    }
    xSemaphoreGive(*(obj->mutex));
    #ifdef BME680_DEBUG
    // for (int i = 0; i < result; i++)
    // {
    //     printf("read[%d] = 0x%02x\n", i, reg_data[i]);
    // }
    // printf("%d bytes read\n", result);
    #endif
    return (int8_t)(!(result > 0));
}
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface)
{
    bme680_t* obj = (bme680_t*)interface;
    const size_t bufLen = len + 1;
    uint8_t buf[bufLen];
    buf[0] = reg_addr;
    memcpy(&(buf[1]), reg_data, len);
    // printf("address 0x%02x\n", reg_addr);
    xSemaphoreTake(*(obj->mutex), portMAX_DELAY);
    // int result = i2c_write_blocking(obj->i2c_inst, BME68X_DEFAULT_ADDRESS, &reg_addr, 1, true);
    // if (result > 0)
    // {
    //     result = i2c_write_blocking(obj->i2c_inst, BME68X_DEFAULT_ADDRESS, reg_data, (size_t)len, false);
    // } 
    int result = i2c_write_blocking(obj->i2c_inst, BME68X_DEFAULT_ADDRESS, buf, bufLen, false);
    xSemaphoreGive(*(obj->mutex));
    // printf("%d bytes written\n", result);
    return (int8_t)(!(result > 0));
}
static void delay_usec(uint32_t us, void *intf_ptr)
{
    (void)intf_ptr;
    sleep_us((uint64_t)us);
}

bool bme680_performReading(bme680_t* obj)
{
    return bme680_endReading(obj);
}

uint32_t bme680_beginReading(bme680_t* obj)
{
    if(obj->meas_start != 0)
    {
        return obj->meas_start + obj->meas_period;
    }
    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &(obj->gas_sensor));
#ifdef BME680_DEBUG

#endif
    if (rslt != BME68X_OK)
    {
        return false;
    }
        
    /* Calculate delay period in microseconds */
    uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(BME68X_FORCED_MODE, &(obj->gas_conf), &(obj->gas_sensor))
                                + ((uint32_t)obj->gas_heatr_conf.heatr_dur * 1000);
    // Serial.print("measure: ");
    // Serial.println(bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf,
    // &gas_sensor)); Serial.print("heater: ");
    // Serial.println((uint32_t)gas_heatr_conf.heatr_dur * 1000);

    obj->meas_start = to_ms_since_boot(get_absolute_time());
    obj->meas_period = delayus_period / 1000;

    return obj->meas_start + obj->meas_period;
}

bool bme680_endReading(bme680_t* obj)
{
    uint32_t meas_end = bme680_beginReading(obj);

    if (meas_end == 0) {
        printf("Meas_end is 0\n");
        return false;
    }

    int remaining_millis = bme680_remainingReadingMillis(obj);

    if (remaining_millis > 0) {
    #ifdef BME680_DEBUG
        // printf("Remaining time: %d\n", remaining_millis);
    #endif
        // sleep_ms(remaining_millis * 2); /* Delay till the measurement is ready */
        vTaskDelay(remaining_millis*2 / portTICK_PERIOD_MS);
    }
    obj->meas_start = 0; /* Allow new measurement to begin */
    obj->meas_period = 0;

#ifdef BME680_DEBUG
#endif

    struct bme68x_data data;
    uint8_t n_fields;

#ifdef BME680_DEBUG
#endif

  int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &(obj->gas_sensor));
#ifdef BME680_DEBUG
#endif
    if (rslt != BME68X_OK)
    {
        printf("Failed to get data: %d\n", rslt);
        return false;
    }
    
    if (n_fields) {
        obj->temperature_raw = data.temperature;
        obj->humidity_raw = data.humidity;
        obj->pressure = data.pressure;

    #ifdef BME680_DEBUG
    #endif
        if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
        obj->gas_resistance_raw = data.gas_resistance;
        } else {
        obj->gas_resistance_raw = 0;
        }
    }
  return true;
}

float bme680_readTemperature(bme680_t* obj)
{
    bme680_performReading(obj);
    return obj->temperature_raw;
}

float bme680_readPressure(bme680_t* obj)
{
    bme680_performReading(obj);
    return obj->pressure;
}

float bme680_readHumidity(bme680_t* obj)
{
    bme680_performReading(obj);
    return obj->humidity_raw;
}

uint32_t bme680_readGas(bme680_t* obj)
{
    bme680_performReading(obj);
    return obj->gas_resistance_raw;
}

float bme680_readAltitude(bme680_t* obj, float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
    float atmospheric = bme680_readPressure(obj) / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

int bme680_remainingReadingMillis(bme680_t* obj)
{
    if (obj->meas_start != 0) {
        /* A measurement is already in progress */
        int remaining_time = (int)obj->meas_period - (to_ms_since_boot(get_absolute_time()) - obj->meas_start);
        return remaining_time < 0 ? reading_complete : remaining_time;
    }
    return reading_not_started;
}