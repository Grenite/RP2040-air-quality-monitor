#include "hardware/i2c.h"
#include <FreeRTOS.h>
#include <semphr.h>
#define SCD4X_I2C_ADDRESS 0x62

typedef struct scd41_t {
    i2c_inst_t* i2c_inst;
    SemaphoreHandle_t* mutex;
    bool init_ok;
    uint16_t co2;
    float temperature;
    float humidity;
    float tOffset;
} scd41_t;

scd41_t* scd41_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl);
void scd41_deinit(scd41_t* obj);
bool scd41_begin(scd41_t* obj);
bool scd41_startPeriodicMeasurement(scd41_t* obj);
bool scd41_readMeasurementTicks(scd41_t* obj, uint16_t* co2, uint16_t* temperature, uint16_t* humidity);
bool scd41_readMeasurement(scd41_t* obj);
bool scd41_stopPeriodicMeasurement(scd41_t* obj);
bool scd41_getTemperatureOffsetTicks(scd41_t* obj, uint16_t* tOffset);
bool scd41_getTemperatureOffset(scd41_t* obj);
bool scd41_setTemperatureOffsetTicks(scd41_t* obj, const uint16_t tOffset);
bool scd41_setTemperatureOffset(scd41_t* obj, float tOffset);
bool scd41_getSensorAltitude(scd41_t* obj, uint16_t* sensorAltitude);
bool scd41_setSensorAltitude(scd41_t* obj, const uint16_t sensorAltitude);
bool scd41_setAmbientPressure(scd41_t* obj, const uint16_t ambientPressure);
bool scd41_performForcedRecalibration(scd41_t* obj, uint16_t targetCo2Concentration, uint16_t* frcCorrection);
bool scd41_getAutomaticSelfCalibration(scd41_t* obj, uint16_t* ascEnabled);
bool scd41_setAutomaticSelfCalibration(scd41_t* obj, uint16_t ascEnabled);
bool scd41_getAutomaticSelfCalibrationInitialPeriod(scd41_t* obj, uint16_t* ascInitialPeriod);
bool scd41_getAutomaticSelfCalibrationStandardPeriod(scd41_t* obj, uint16_t* ascStandardPeriod);
bool scd41_setAutomaticSelfCalibrationInitialPeriod(scd41_t* obj, uint16_t ascInitialPeriod);
bool scd41_setAutomaticSelfCalibrationStandardPeriod(scd41_t* obj, uint16_t ascStandardPeriod);
bool scd41_startLowPowerPeriodicMeasurement(scd41_t* obj);
bool scd41_getDataReadyFlag(scd41_t* obj, bool* dataReadyFlag);
bool scd41_persistSettings(scd41_t* obj);
bool scd41_getSerialNumber(scd41_t* obj, uint16_t* serial0, uint16_t* serial1, uint16_t* serial2);
bool scd41_performSelfTest(scd41_t* obj, uint16_t* sensorStatus);
bool scd41_performFactoryReset(scd41_t* obj);
bool scd41_reinit(scd41_t* obj);
bool scd41_measureSingleShot(scd41_t* obj, bool blocking);
bool scd41_measureSingleShotRhtOnly(scd41_t* obj);
bool scd41_powerDown(scd41_t* obj);
bool scd41_wakeUp(scd41_t* obj);
