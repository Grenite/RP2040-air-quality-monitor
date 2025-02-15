#include "BME68xBSec.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <bsec_interface_multi.h>

/*
 *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
 *	Please update the offset value after testing this on your product 
 */
#define TEMP_OFFSET_LP		(1.3255f + 0.79f)
#define TEMP_OFFSET_ULP		(0.466f)


//set config is required for lp mode
//3.3v 3s sampling rate with 4 days of calibration data
const static uint8_t bsec_config[] = {
                                  #include "Bosch-BSEC2-Library/src/config/bme688/bme688_sel_33v_3s_4d/bsec_selectivity.txt"
                                };
static uint8_t workBuffer[BSEC_MAX_WORKBUFFER_SIZE];

bsec_t *bsec_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl)
{
    bsec_t *result = (bsec_t*)malloc(sizeof(bsec_t));
    memset(result, 0, sizeof(bsec_t));
    result->bsec_instance = malloc(bsec_get_instance_size_m());
    result->bsec_sensors_count = BSEC_MAX_PHYSICAL_SENSOR;
    result->sensor = bme680_init(i2c_inst, mutex, init_i2c, baudrate, sda, scl);
    return result;
}

bool bsec_begin(bsec_t *obj)
{
    bsec_library_return_t lib_result = bsec_init_m(obj->bsec_instance);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_init_m failed: %d\n", lib_result);
        return false;
    }
    if (!bme680_begin(obj->sensor))
    {
        printf("bme680_begin failed!\n");
        return false;
    }
    lib_result = bsec_set_configuration_m(obj->bsec_instance, bsec_config, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_set_configuration_m failed: %d\n", lib_result);
        return false;
    }
    bsec_sensor_configuration_t requested_sensors[4];
    uint8_t n_requested_virtual_sensors = 4;
    requested_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_sensors[0].sample_rate =  BSEC_SAMPLE_RATE_LP;
    requested_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_sensors[1].sample_rate =  BSEC_SAMPLE_RATE_LP;
    requested_sensors[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_sensors[2].sample_rate =  BSEC_SAMPLE_RATE_LP;
    requested_sensors[3].sensor_id = BSEC_OUTPUT_COMPENSATED_GAS;
    requested_sensors[3].sample_rate =  BSEC_SAMPLE_RATE_LP;
    lib_result = bsec_update_subscription_m(obj->bsec_instance, requested_sensors, n_requested_virtual_sensors, obj->bsec_sensors, &obj->bsec_sensors_count);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_update_subscription_m failed: %d\n", lib_result);
        return false;
    }
    const uint64_t ts = to_us_since_boot(get_absolute_time())*INT64_C(1000);
    lib_result = bsec_sensor_control_m(obj->bsec_instance, (int64_t)ts, &obj->bsec_config);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_sensor_control_m error: %d, %llu\n", lib_result, ts);
        return false;
    }
    return true;
}

void bsec_deinit(bsec_t *obj)
{
    bme680_deinit(obj->sensor);
    free(obj);
}

bool bsec_read_data(bsec_t *obj)
{
    bsec_library_return_t lib_result = bsec_sensor_control_m(obj->bsec_instance, obj->bsec_config.next_call, &obj->bsec_config);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_sensor_control_m error: %d, %llu, %llu\n", lib_result, obj->bsec_config.next_call, obj->bsec_config.next_call);
        return false;
    }
    bme680_setHumidityOversampling(obj->sensor, obj->bsec_config.humidity_oversampling);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bme680_setPressureOversampling(obj->sensor, obj->bsec_config.pressure_oversampling);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bme680_setTemperatureOversampling(obj->sensor, obj->bsec_config.temperature_oversampling);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bme680_setGasHeater(obj->sensor, obj->bsec_config.heater_temperature, obj->bsec_config.heater_duration);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    if (!obj->bsec_config.trigger_measurement)
    {
        return true;
    }
    bme680_performReading(obj->sensor);
    bsec_input_t input[5];
    uint8_t n_input = 5;
    const uint64_t ts = to_us_since_boot(get_absolute_time())*INT64_C(1000);
    input[0].sensor_id = BSEC_INPUT_GASRESISTOR;
    input[0].signal = obj->sensor->gas_resistance_raw;
    input[0].time_stamp= ts;   
    input[1].sensor_id = BSEC_INPUT_TEMPERATURE;   
    input[1].signal = obj->sensor->temperature_raw;   
    input[1].time_stamp= ts;   
    input[2].sensor_id = BSEC_INPUT_HUMIDITY;
    input[2].signal = obj->sensor->humidity_raw;
    input[2].time_stamp= ts;   
    input[3].sensor_id = BSEC_INPUT_PRESSURE;
    input[3].signal = obj->sensor->pressure;
    input[3].time_stamp= ts;  
    input[4].sensor_id = BSEC_INPUT_HEATSOURCE;
    input[4].signal = TEMP_OFFSET_LP;
    input[4].time_stamp= ts;  
    bsec_output_t output[obj->bsec_sensors_count];
    uint8_t n_output = obj->bsec_sensors_count;
    lib_result = bsec_do_steps_m(obj->bsec_instance, input, n_input, output, &n_output);
    if (lib_result != BSEC_OK)
    {
        printf("bsec_do_steps_m failed: %d\n", lib_result);
        return false;
    }
    for(int i = 0; i < n_output; i++)
    {   
        switch(output[i].sensor_id)
        {
            case BSEC_OUTPUT_IAQ:
                // Retrieve the IAQ results from output[i].signal
                // and do something with the data
                obj->voc_index = output[i].signal;
                break;
            case BSEC_OUTPUT_COMPENSATED_GAS:
                // Retrieve the static IAQ results from output[i].signal
                // and do something with the data
                obj->gas_resistance = output[i].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                obj->temperature = output[i].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                obj->humidity = output[i].signal;
                break;
            default:
                break;
        }
    }
    return true;
}

int64_t bsec_get_next_sample_time(bsec_t *obj)
{
    return obj->bsec_config.next_call;
}

bool bsec_sample_ready(bsec_t *obj)
{
    const int64_t time = (int64_t)(to_us_since_boot(get_absolute_time())*INT64_C(1000));
    const int64_t next_time = bsec_get_next_sample_time(obj);
    const bool result = time >= next_time;
    // printf("ready? %lld vs %lld %d\n", time, next_time, result);
    return  result;
}