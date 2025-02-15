#pragma once
#include "BME680.h"
#include <bsec_datatypes.h>
typedef struct bsec_t {
    bme680_t *sensor;
    uint32_t voc_index;
    float temperature;
    float humidity;
    uint32_t gas_resistance;
    //private data for bsec multi algorithm
    void* bsec_instance;
    bsec_sensor_configuration_t bsec_sensors[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t bsec_sensors_count;
    bsec_bme_settings_t bsec_config;
}bsec_t;

bsec_t *bsec_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl);
bool bsec_begin(bsec_t *obj);
void bsec_deinit(bsec_t *obj);

bool bsec_read_data(bsec_t *obj);
int64_t bsec_get_next_sample_time(bsec_t *obj);
bool bsec_sample_ready(bsec_t *obj);