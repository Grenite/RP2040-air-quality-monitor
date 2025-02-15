#include "SCD41.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/pio.h>
static int i2cWrite(scd41_t* obj, const uint8_t addr, const uint8_t *buf, const size_t buf_len, const bool nostop)
{
    xSemaphoreTake(*obj->mutex, portMAX_DELAY);
    int result = i2c_write_blocking(obj->i2c_inst, addr, buf, buf_len, nostop);
    xSemaphoreGive(*obj->mutex);
}
static int i2cWriteRead(scd41_t* obj, const uint8_t addr, const uint16_t reg_addr, uint8_t *buf, const size_t buf_len, const bool nostop)
{
    xSemaphoreTake(*obj->mutex, portMAX_DELAY);
    const uint8_t cmd[2] = {
        (reg_addr & 0xFF00) >> 8,
        (reg_addr & 0x00FF)
    };
    int writeResult = i2c_write_blocking(obj->i2c_inst, addr, cmd, 2, true);
    int result = writeResult;
    if (result > 0)
    {
        result = i2c_read_blocking(obj->i2c_inst, addr, buf, buf_len, nostop);
    }
    xSemaphoreGive(*obj->mutex);
    return result;
}
static int i2cRead(scd41_t* obj, const uint8_t addr, uint8_t *buf, const size_t buf_len, const bool nostop)
{
    xSemaphoreTake(*obj->mutex, portMAX_DELAY);
    int result = i2c_read_blocking(obj->i2c_inst, addr, buf, buf_len, nostop);
    xSemaphoreGive(*obj->mutex);
    return result;
}

static void delay_ms(const uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
typedef enum CrcPolynomial {
    CRC31_00 = 0x0,
    CRC31_ff = 0x1,
}CrcPolynomial;

static uint8_t generateCRCGeneric(const uint8_t* data, size_t count, uint8_t init,
                           uint8_t polynomial) {
    uint8_t crc = init;

    /* calculates 8-Bit checksum with given polynomial */
    for (size_t current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (uint8_t crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ polynomial;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static uint8_t generateCRC31_ff(const uint8_t* data, size_t count) {
    return generateCRCGeneric(data, count, 0xff, 0x31);
}

static uint8_t generateCRC31_00(const uint8_t* data, size_t count) {
    return generateCRCGeneric(data, count, 0x00, 0x31);
}

static uint8_t generateCRC(const uint8_t* data, size_t count, CrcPolynomial type) {
    if (CRC31_00 == type) {
        return generateCRC31_00(data, count);
    }
    return generateCRC31_ff(data, count);
}

static bool verifyCRC(const uint8_t* data, size_t count, CrcPolynomial type) {
    if (count % 3 != 0) return false;
    for (int i = 0; i < count; i+=3)
    {
        const uint8_t crc = generateCRC(&data[i], 2, type);
        if (crc != data[i + 2]) return false;
    }
    return true;
}

scd41_t* scd41_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl)
{
    scd41_t *data = (scd41_t*)malloc(sizeof(scd41_t));
    data->i2c_inst = i2c_inst;
    data->mutex = mutex;
    data->init_ok = false;
    data->co2 = 0;
    data->temperature = 0;
    data->humidity = 0;
    if (init_i2c)
    {
        i2c_init(i2c_inst, baudrate);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }
    return data;
}
void scd41_deinit(scd41_t* obj)
{
    free(obj);
}
bool scd41_begin(scd41_t* obj)
{
    obj->init_ok = true;
    return true;
}
bool scd41_startPeriodicMeasurement(scd41_t* obj)
{
    const uint8_t cmd[2] = {0x21, 0xb1};
    const int result = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return (result > 0);
}
bool scd41_readMeasurementTicks(scd41_t* obj, uint16_t* co2, uint16_t* temperature, uint16_t* humidity)
{
    const uint16_t cmd = 0xEC05;
    uint8_t buf[9];
    int result = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 9, false);
    if (result <= 0 || !verifyCRC(buf, 9, CRC31_ff))
    {
        return false;
    }
    if (co2 != NULL)*co2 = ((uint16_t)buf[0]) << 8 | (uint16_t)buf[1];
    if (temperature != NULL) *temperature = ((uint16_t)buf[3]) << 8 | (uint16_t)buf[4];
    if (humidity != NULL) *humidity = ((uint16_t)buf[6]) << 8 | (uint16_t)buf[7];
    return true;
}
bool scd41_readMeasurement(scd41_t* obj)
{
    uint16_t temperature_tick;
    uint16_t humidity_tick;
    if (!scd41_readMeasurementTicks(obj, &obj->co2, &temperature_tick, &humidity_tick)) return false;
    obj->temperature = (float)temperature_tick * 175.0 / 65535.0 - 45.0;
    obj->humidity = (float)humidity_tick * 100.0 / 65535.0;
    return true;
}
bool scd41_stopPeriodicMeasurement(scd41_t* obj)
{
    const uint8_t cmd[2] = {0x3f, 0x86};
    int result = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return result > 0;
}
bool scd41_getTemperatureOffsetTicks(scd41_t* obj, uint16_t* tOffset)
{
    uint8_t buf[3] = {0};
    const uint16_t cmd = 0x2318;
    int result = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 3, false);
    if (result <= 0 || !verifyCRC(buf, 3, CRC31_ff))
    {
        return false;
    }
    if (tOffset!= NULL) *tOffset = ((uint16_t)buf[0]) << 8 | (uint16_t)buf[1];
    return true;
}
bool scd41_getTemperatureOffset(scd41_t* obj)
{
    uint16_t tOffset_tick;
    if (!scd41_getTemperatureOffsetTicks(obj, &tOffset_tick))
    {
        return false;
    }
    obj->tOffset = (float)tOffset_tick * 175.0 / 65536.0;
    return true;
}
bool scd41_setTemperatureOffsetTicks(scd41_t* obj, const uint16_t tOffset)
{
    const uint8_t data[2] = {
        (tOffset & 0xFF00) >> 8,
        (tOffset & 0x00FF)
    };
    const uint8_t buffer[5] = {
        0x24,
        0x1D,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    int result = i2cWrite(obj, SCD4X_I2C_ADDRESS, buffer, 5, false);
    return result > 0; 
}
bool scd41_setTemperatureOffset(scd41_t* obj, float tOffset)
{
    const uint16_t tOffsetTicks = ((float)tOffset * 65536.0 / 175.0 + 0.5f);
    return scd41_setTemperatureOffsetTicks(obj, tOffsetTicks);
}
bool scd41_getSensorAltitude(scd41_t* obj, uint16_t* sensorAltitude)
{
    uint8_t buffer[3];
    const uint16_t cmd = 0x2322;
    int result = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buffer, 3, false);
    if (result <= 0 || !verifyCRC(buffer, 3, CRC31_ff))
    {
        return false;
    }
    if (sensorAltitude != NULL) *sensorAltitude = ((uint16_t)buffer[0]) << 8 | (uint16_t)buffer[1];
    return true;
}
bool scd41_setSensorAltitude(scd41_t* obj, const uint16_t sensorAltitude)
{
    const uint8_t data[2] = {
        (sensorAltitude & 0xFF00) >> 8,
        (sensorAltitude & 0x00FF)
    };
    const uint8_t buffer[5] = {
        0x24,
        0x27,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    int result = i2cWrite(obj, SCD4X_I2C_ADDRESS, buffer, 5, false);
    return result > 0;
}
bool scd41_setAmbientPressure(scd41_t* obj, const uint16_t ambientPressure)
{
    const uint8_t data[2] = {
        (ambientPressure & 0xFF00) >> 8,
        (ambientPressure & 0x00FF)
    };
    const uint8_t buffer[5] = {
        0x24,
        0x27,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    int result = i2cWrite(obj, SCD4X_I2C_ADDRESS, buffer, 5, false);
    return result > 0;
}
bool scd41_performForcedRecalibration(scd41_t* obj, uint16_t targetCo2Concentration, uint16_t* frcCorrection)
{
    const uint8_t data[2] = {
        (targetCo2Concentration & 0xFF00) >> 8,
        (targetCo2Concentration & 0x00FF)
    };
    const uint8_t tx_buffer[5] = {
        0x36,
        0x2F,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, tx_buffer, 5, false);
    if (sendResult <= 0 ) return false;
    delay_ms(400);
    uint8_t rx_buffer[3] = {0};
    int recvResult = i2cRead(obj, SCD4X_I2C_ADDRESS, rx_buffer, 3, false);
    if (recvResult <= 0 || !verifyCRC(rx_buffer, 3, CRC31_ff))
    {
        return false;
    }
    if (frcCorrection != NULL) *frcCorrection = ((uint16_t)rx_buffer[0]) << 8 | (uint16_t)rx_buffer[1];
    return true;
}
bool scd41_getAutomaticSelfCalibration(scd41_t* obj, uint16_t* ascEnabled)
{
    const uint16_t cmd = 0x2313;
    uint8_t buf[3] = {0};
    int result = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 3, false);
    if (result <= 0 || !verifyCRC(buf, 3, CRC31_ff))
    {
        return false;
    }
    if (ascEnabled != NULL) *ascEnabled = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return true;
}
bool scd41_setAutomaticSelfCalibration(scd41_t* obj, uint16_t ascEnabled)
{
    const uint8_t data[2] = {
        (ascEnabled & 0xFF00) >> 8,
        (ascEnabled & 0x00FF)
    };
    const uint8_t tx_buffer[5] = {
        0x36,
        0x2F,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, tx_buffer, 5, false);
    return sendResult > 0;
}
bool scd41_getAutomaticSelfCalibrationInitialPeriod(scd41_t* obj, uint16_t* ascInitialPeriod)
{
    const uint16_t cmd = 0x2340;
    uint8_t buf[3] = {0};
    const int sendResult = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 3, false);
    if (sendResult <= 0 || !verifyCRC(buf, 3, CRC31_ff))
    {
        return false;
    }
    if (ascInitialPeriod != NULL) *ascInitialPeriod = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return true;
}
bool scd41_getAutomaticSelfCalibrationStandardPeriod(scd41_t* obj, uint16_t* ascStandardPeriod)
{
    const uint16_t cmd = 0x234B;
    uint8_t buf[3] = {0};
    const int sendResult = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 3, false);
    if (sendResult <= 0 || !verifyCRC(buf, 3, CRC31_ff))
    {
        return false;
    }
    if (ascStandardPeriod != NULL) *ascStandardPeriod = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return true;
}
bool scd41_setAutomaticSelfCalibrationInitialPeriod(scd41_t* obj, uint16_t ascInitialPeriod)
{
    const uint8_t data[2] = {
        (ascInitialPeriod & 0xFF00) >> 8,
        (ascInitialPeriod & 0x00FF)
    };
    const uint8_t tx_buffer[5] = {
        0x24,
        0x45,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, tx_buffer, 5, false);
    return sendResult > 0;
}
bool scd41_setAutomaticSelfCalibrationStandardPeriod(scd41_t* obj, uint16_t ascStandardPeriod)
{
    const uint8_t data[2] = {
        (ascStandardPeriod & 0xFF00) >> 8,
        (ascStandardPeriod & 0x00FF)
    };
    const uint8_t tx_buffer[5] = {
        0x24,
        0x4E,
        data[0],
        data[1],
        generateCRC(data, 2, CRC31_ff)
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, tx_buffer, 5, false);
    return sendResult > 0;
}
bool scd41_startLowPowerPeriodicMeasurement(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0xE4,
        0xB8
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_getDataReadyFlag(scd41_t* obj, bool* dataReadyFlag)
{
    const uint16_t cmd = 0xE4B8;
    uint8_t buf[3] = {0};
    const int sendResult = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, cmd, buf, 3, false);
    if (sendResult <= 0 || !verifyCRC(buf, 3, CRC31_ff))
    {
        return false;
    }
    const uint16_t localDataReady = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    if (dataReadyFlag != NULL) *dataReadyFlag = (localDataReady & 0x07FF) != 0;
    return true;
}
bool scd41_persistSettings(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x36,
        0x15
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_getSerialNumber(scd41_t* obj, uint16_t* serial0, uint16_t* serial1, uint16_t* serial2)
{
    uint8_t buf[9] = {0};
    const int sendResult = i2cWriteRead(obj, SCD4X_I2C_ADDRESS, 0x3682, buf, 9, false);
    if (sendResult <= 0) return false;
    // printf("Get Serial number data %d\n", sendResult);
    // for (int i = 0; i < sendResult; i++)
    // {
    //     printf("buf[%d]: %02x\n", i, (unsigned int)buf[i]);
    // }
    if (!verifyCRC(buf, 3, CRC31_ff)) return false;
    if (serial0 != NULL) *serial0 = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    // printf("Serial1 ok\n");
    if (sendResult > 3)
    {
        if (!verifyCRC(&buf[3], 3, CRC31_ff)) return false;
        if (serial1 != NULL) *serial1 = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];
        // printf("Serial2 ok\n");
        if (sendResult > 6)
        {
            if (!verifyCRC(&buf[6], 3, CRC31_ff)) return false;
            if (serial2 != NULL) *serial2 = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];
            // printf("Serial3 ok\n");
        }
    }

    return true;
}
bool scd41_performSelfTest(scd41_t* obj, uint16_t* sensorStatus)
{
    const uint8_t tx_buffer[2] = {0x36, 0x39};
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, tx_buffer, 2, false);
    if (sendResult <= 0 ) return false;
    delay_ms(10000);
    uint8_t rx_buffer[3] = {0};
    int recvResult = i2cRead(obj, SCD4X_I2C_ADDRESS, rx_buffer, 3, false);
    if (recvResult <= 0 || !verifyCRC(rx_buffer, 3, CRC31_ff))
    {
        return false;
    }
    if (sensorStatus != NULL) *sensorStatus = ((uint16_t)rx_buffer[0]) << 8 | (uint16_t)rx_buffer[1];
    return true;
}
bool scd41_performFactoryReset(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x36,
        0x32
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_reinit(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x36,
        0x46
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_measureSingleShot(scd41_t* obj, bool blocking)
{
    const uint8_t cmd[2] = {
        0x21,
        0x9D
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    if (blocking) delay_ms(5000);
    return sendResult > 0;
}
bool scd41_measureSingleShotRhtOnly(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x21,
        0x96
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_powerDown(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x36,
        0xE0
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return sendResult > 0;
}
bool scd41_wakeUp(scd41_t* obj)
{
    const uint8_t cmd[2] = {
        0x36,
        0xE0
    };
    const int sendResult = i2cWrite(obj, SCD4X_I2C_ADDRESS, cmd, 2, false);
    return true;
}