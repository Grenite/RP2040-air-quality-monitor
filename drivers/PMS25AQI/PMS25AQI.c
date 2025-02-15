#include "PMS25AQI.h"
#include <stdlib.h>
#include <string.h>
#include <hardware/pio.h>
PMS25AQI* pms25aqi_init (i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, bool init_i2c, unsigned int baudrate, int sda, int scl)
{
    struct PMS25AQI* state = (struct PMS25AQI*)malloc(sizeof(struct  PMS25AQI));
    if (init_i2c)
    {
        i2c_init(i2c_inst, baudrate);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }
    state->i2c_inst = i2c_inst;
    state->mutex = mutex;
    return state;
}
void pms25aqi_deinit(PMS25AQI* obj)
{
    free(obj);
}
bool pms25aqi_read(PMS25AQI* obj, PM25_AQI_Data *data)
{
    struct PMSAQIdata  result;
    bool ok = true;
    uint8_t buffer[32];
    xSemaphoreTake(*obj->mutex, portMAX_DELAY);
    if (i2c_read_blocking(obj->i2c_inst, PMSA003I_I2CADDR_DEFAULT, buffer, sizeof(buffer), false) != 32)
    {
        ok = false;
    }
    else if (buffer[0] != 0x42)
    {
        ok = false;
    }
    else
    {
        uint16_t csum = 0;
        for (uint8_t i = 0; i < 30; i++) 
        {
            csum += buffer[i];
        }
        // The data comes in endian'd, this solves it so it works on all platforms
        uint16_t buffer_u16[15];
        for (uint8_t i = 0; i < 15; i++)
        {
            buffer_u16[i] = buffer[2 + i * 2 + 1];
            buffer_u16[i] += (buffer[2 + i * 2] << 8);
        }
        memcpy((void*)data, (void*)buffer_u16, 30);
        if (csum != data->checksum)
        {
            ok = false;
        }
    }
    xSemaphoreGive(*obj->mutex);
    return ok;
}