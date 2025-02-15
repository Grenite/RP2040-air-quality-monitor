#ifndef __PMS25AQI_H__
#define __PMS25AQI_H__

#include "hardware/i2c.h"
#include "I2C-Mutex.h"
//referene: https://github.com/adafruit/Adafruit_PM25AQI
// the i2c address
#define PMSA003I_I2CADDR_DEFAULT 0x12 ///< PMSA003I has only one I2C address

/**! Structure holding Plantower's standard packet **/
typedef struct PMSAQIdata {
    uint16_t framelen;       ///< How long this data chunk is
    uint16_t pm10_standard,  ///< Standard PM1.0
        pm25_standard,       ///< Standard PM2.5
        pm100_standard;      ///< Standard PM10.0
    uint16_t pm10_env,       ///< Environmental PM1.0
        pm25_env,            ///< Environmental PM2.5
        pm100_env;           ///< Environmental PM10.0
    uint16_t particles_03um, ///< 0.3um Particle Count
        particles_05um,      ///< 0.5um Particle Count
        particles_10um,      ///< 1.0um Particle Count
        particles_25um,      ///< 2.5um Particle Count
        particles_50um,      ///< 5.0um Particle Count
        particles_100um;     ///< 10.0um Particle Count
    uint16_t unused;         ///< Unused
    uint16_t checksum;       ///< Packet checksum
} PM25_AQI_Data;

typedef struct PMS25AQI {
    i2c_inst_t* i2c_inst;
    SemaphoreHandle_t *mutex;
}PMS25AQI;

PMS25AQI* pms25aqi_init (i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex,  bool init_i2c, unsigned int baudrate, int sda, int scl);
void pms25aqi_deinit(PMS25AQI* obj);
bool pms25aqi_read(PMS25AQI* obj, PM25_AQI_Data *data);

#endif