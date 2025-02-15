#ifndef __SGP40_H__
#define __SGP40_H__

#include "hardware/i2c.h"
#include "I2C-Mutex.h"
#include "sensirion_arch_config.h"
#include "sensirion_gas_index_algorithm.h"
//https://github.com/adafruit/Adafruit_SGP40
#define SGP40_I2CADDR_DEFAULT 0x59 ///< SGP40 has only one I2C address

// commands and constants
#define SGP40_FEATURESET 0x0020    ///< The required set for this library
#define SGP40_CRC8_POLYNOMIAL 0x31 ///< Seed for SGP40's CRC polynomial
#define SGP40_CRC8_INIT 0xFF       ///< Init value for CRC
#define SGP40_WORD_LEN 2           ///< 2 bytes per word

typedef struct sgp40 {
    i2c_inst_t* i2c_inst;
    SemaphoreHandle_t* mutex;
    uint16_t serialNumber[3];
    uint16_t featureset;
    GasIndexAlgorithmParams voc_algorithm_params;
    bool init_ok;
} sgp40_t;

sgp40_t *sgp40_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, float sampling_interval, bool init_i2c, unsigned int baudrate, int sda, int scl);
void sgp40_deinit(sgp40_t* obj);

bool sgp40_begin(sgp40_t* obj);
bool sgp40_selfTest(sgp40_t* obj);
bool sgp40_softReset(sgp40_t* obj);
bool sgp40_heaterOff(sgp40_t* obj);
uint16_t sgp40_measureRaw(sgp40_t* obj, float temperature, float humidity);
int32_t sgp40_measureVOCIndex(sgp40_t* obj, float temperature, float humidity);


#endif