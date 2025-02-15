#include "SGP40.h"
#include <hardware/pio.h>
// #include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
static uint8_t generateCRC(uint8_t *data, uint8_t datalen) 
{
  // calculates 8-Bit checksum with given polynomial
  uint8_t crc = SGP40_CRC8_INIT;

  for (uint8_t i = 0; i < datalen; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ SGP40_CRC8_POLYNOMIAL;
      else
        crc <<= 1;
    }
  }
  return crc;
}

static bool sgp40_readWordFromCommand(sgp40_t* obj, 
    const uint8_t command[],
    uint8_t commandLength,
    uint16_t delayms, 
    uint16_t *readdata,
    uint8_t readlen)
{
    xSemaphoreTake(*obj->mutex, portMAX_DELAY);
    int written = i2c_write_blocking(obj->i2c_inst, SGP40_I2CADDR_DEFAULT, command, commandLength, true);
    if ( written <= 0)
    {
        // printf("Failed to write to command: %d\n", written);
        xSemaphoreGive(*obj->mutex);
        return false;
    }
    if (readlen == 0 || readdata == NULL)
    {
        xSemaphoreGive(*obj->mutex);
        return true;
    }
    uint8_t replylen = readlen * (SGP40_WORD_LEN + 1);
    uint8_t replybuffer[replylen];
    memset(replybuffer, 0, replylen);
    vTaskDelay(delayms / portTICK_PERIOD_MS);
    // sleep_ms(delayms);

    int read = i2c_read_blocking(obj->i2c_inst, SGP40_I2CADDR_DEFAULT, replybuffer, replylen, false);
    if (read <= 0)
    {
        // printf("Failed to read from command: %d\n", read);
        xSemaphoreGive(*obj->mutex);
        return false;
    }
    for (uint8_t i = 0; i < readlen; i++)
    {
        uint8_t crc = generateCRC(replybuffer + i * 3, 2);
        if (crc != replybuffer[i*3 + 2])
        {
            // printf("Crc fail: crc 0x%02x, reply 0x%02x, index: %u, read: %d, value: %02x %02x\n", crc, replybuffer[i*3 + 2], (unsigned int)i, read, replybuffer[i*3 + 0], replybuffer[i*3 + 1]);
            xSemaphoreGive(*obj->mutex);
            return false;
        }
        readdata[i] = replybuffer[i*3];
        readdata[i] <<= 8;
        readdata[i] |= replybuffer[i*3 + 1];
    }
    xSemaphoreGive(*obj->mutex);
    return true;
}

sgp40_t *sgp40_init(i2c_inst_t* i2c_inst, SemaphoreHandle_t *mutex, float sampling_interval,  bool init_i2c, unsigned int baudrate, int sda, int scl)
{
    sgp40_t *data = (sgp40_t*)malloc(sizeof(sgp40_t));
    data->i2c_inst = i2c_inst;
    data->mutex = mutex;
    data->init_ok = false;
    if (init_i2c)
    {
        i2c_init(i2c_inst, baudrate);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }
    GasIndexAlgorithm_init_with_sampling_interval(&data->voc_algorithm_params, 0, sampling_interval);
    return data;
}

void sgp40_deinit(sgp40_t* obj)
{
    free(obj);
}

bool sgp40_begin(sgp40_t* obj)
{
    uint8_t command[2] = {
        0x36, 0x82
    };
    if (!sgp40_readWordFromCommand(obj, command, 2, 5, obj->serialNumber, 3))
    {
        // printf("Failed to begin from first command\n");
        return false;
    }
    command[0] = 0x20;
    command[1] = 0x2F;
    if (!sgp40_readWordFromCommand(obj, command, 2, 5, &obj->featureset, 1))
    {
        // printf("Failed to begin from second command\n");
        return false;
    }
    if (!sgp40_selfTest) return false;
    obj->init_ok = true;
    return true;
}

bool sgp40_selfTest(sgp40_t* obj)
{
    uint8_t command[2] = {0x28, 0x0E};
    uint16_t reply = 0x0;
    if(!sgp40_readWordFromCommand(obj, command, 2, 320, &reply, 1))
    {
        // printf("Self test failure\n");
        return false;
    }
    if (reply != 0xD400)
    {
        // printf("Self test reply wrong: %04x", reply);
        return false;
    }
    return true;
}

bool sgp40_softReset(sgp40_t* obj)
{
    const uint8_t command[2] = {0x00, 0x06};
    return sgp40_readWordFromCommand(obj, command, 2, 10, NULL, 0);
}

bool sgp40_heaterOff(sgp40_t* obj)
{
    const uint8_t command[2] = {0x36, 0x15};
    return sgp40_readWordFromCommand(obj, command, 2, 10, NULL, 0);
}

uint16_t sgp40_measureRaw(sgp40_t* obj, float temperature, float humidity)
{
    uint8_t command[8];
    uint16_t reply;

    command[0] = 0x26;
    command[1] = 0x0F;

    uint16_t rhticks = (uint16_t)((humidity * 65535) / 100 + 0.5);
    command[2] = rhticks >> 8;
    command[3] = rhticks & 0xFF;
    command[4] = generateCRC(command + 2, 2);
    uint16_t tempticks = (uint16_t)(((temperature + 45) * 65535) / 175);
    command[5] = tempticks >> 8;
    command[6] = tempticks & 0xFF;
    command[7] = generateCRC(command + 5, 2);

    if (!sgp40_readWordFromCommand(obj, command, 8, 100, &reply, 1))
        return 0x0;

    return reply;
}

int32_t sgp40_measureVOCIndex(sgp40_t* obj, float temperature, float humidity)
{
    int32_t voc_index;
    const uint16_t sraw = sgp40_measureRaw(obj, temperature, humidity);

    GasIndexAlgorithm_process(&obj->voc_algorithm_params, sraw, &voc_index);
    return voc_index;
}