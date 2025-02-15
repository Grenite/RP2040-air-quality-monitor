#ifndef PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS
#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS (1000)
#endif
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include <gfx.h>

#include "src/gdriver/gdriver.h"
#include "src/ginput/ginput_driver_mouse.h"

#include "drivers/PMS25AQI/PMS25AQI.h"
#include "drivers/SGP40/SGP40.h"
#include "drivers/BME688/BME680.h"
#include "drivers/BME688/BME68xBSEC.h"
#include "drivers/I2C-Mutex/I2C-Mutex.h"
#include "drivers/SCD41/SCD41.h"
#include "gwin_widget.h"

// Priorities of our threads - higher numbers are higher priority
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )
#define WORKER_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE*2
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define WORKER_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static gFont	font;
static gFont    numberFont;
static gFont    labelFont;
static gCoord	swidth, sheight;
static gCoord	bHeight;

SemaphoreHandle_t i2c_mutex;
static scd41_t *scd41;
static sgp40_t *sgp40;
static bsec_t *bme688;
static PMS25AQI *pms25;
static TaskHandle_t poll_task;
static TaskHandle_t button_task_handle;

static GSourceHandle			gs;
static GListener				gl;

static SemaphoreHandle_t data_mutex;
static PM25_AQI_Data pm25Data;
static int sgp40_voc_index;
static bool pm25_data_ready;
static bool bme688_data_ready;
static bool scd41_data_ready;
static bool sgp40_data_ready;

typedef struct data_widget {
    GHandle name;
    GHandle data;
} data_widget;
typedef struct scd41_labels{
    GHandle name;
    data_widget co2;
    data_widget temperature;
    data_widget humidity;
} scd41_labels;

typedef struct sgp40_labels{
    GHandle name;
    data_widget voc_index;
} sgp40_labels;

typedef struct bme688_labels{
    GHandle name;
    data_widget temperature;
    data_widget humidity;
    data_widget pressure;
    data_widget voc_index;
}bme688_labels;

typedef struct pm25_labels {
    GHandle name;
    GHandle name_page_two;
    data_widget pm10_standard;
    data_widget pm25_standard;
    data_widget pm100_standard;
    data_widget pm10_env;
    data_widget pm25_env;
    data_widget pm100_env;
    data_widget particles_03;
    data_widget particles_05;
    data_widget particles_10;
    data_widget particles_25;
    data_widget particles_50;
    data_widget particles_100;
}pm25_labels;

static scd41_labels scd41_handle;
static sgp40_labels sgp40_handle;
static bme688_labels bme688_handle;
static pm25_labels pm25_handle;
static GHandle right_button_handle;
static GHandle left_button_handle;

GHandle *page_one[] = {
    &scd41_handle.name,
    &scd41_handle.co2.name,
    &scd41_handle.co2.data,
    &scd41_handle.temperature.name,
    &scd41_handle.temperature.data,
    &scd41_handle.humidity.name,
    &scd41_handle.humidity.data,
    &sgp40_handle.name,
    &sgp40_handle.voc_index.name,
    &sgp40_handle.voc_index.data,
    &bme688_handle.name,
    &bme688_handle.temperature.name,
    &bme688_handle.temperature.data,
    &bme688_handle.humidity.name,
    &bme688_handle.humidity.data,
    &bme688_handle.pressure.name,
    &bme688_handle.pressure.data,
    &bme688_handle.voc_index.name,
    &bme688_handle.voc_index.data,
    &pm25_handle.name,
    &pm25_handle.pm10_standard.name,
    &pm25_handle.pm10_standard.data,
    &pm25_handle.pm25_standard.name,
    &pm25_handle.pm25_standard.data,
    &pm25_handle.pm100_standard.name,
    &pm25_handle.pm100_standard.data,
    &pm25_handle.pm10_env.name,
    &pm25_handle.pm10_env.data,
    &pm25_handle.pm25_env.name,
    &pm25_handle.pm25_env.data,
    &pm25_handle.pm100_env.name,
    &pm25_handle.pm100_env.data,
};

GHandle *page_two[] = {
    &pm25_handle.particles_03.name,
    &pm25_handle.particles_03.data,
    &pm25_handle.particles_05.name,
    &pm25_handle.particles_05.data,
    &pm25_handle.particles_10.name,
    &pm25_handle.particles_10.data,
    &pm25_handle.particles_25.name,
    &pm25_handle.particles_25.data,
    &pm25_handle.particles_50.name,
    &pm25_handle.particles_50.data,
    &pm25_handle.particles_100.name,
    &pm25_handle.particles_100.data,
    &pm25_handle.name_page_two
};
static int page;
GHandle **pages[] = {
    page_one,
    page_two
};
int page_sizes[] = {
    sizeof(page_one) / sizeof(page_one[0]),
    sizeof(page_two) / sizeof(page_two[0])
};
const size_t max_pages = sizeof(pages) / sizeof(pages[0]);

const static GMouseCalibration calibrationData = {
    .ax = 0.069321,
    .bx = -0.000593,
    .cx = -26.161659,
    .ay = -0.000528,
    .by = 0.091320,
    .cy = -6.591794
};

gBool LoadMouseCalibration(unsigned instance, void *data, gMemSize sz)
{
    memcpy(data, &calibrationData, sizeof(GMouseCalibration));
    return gTrue;
}

static void switch_page(int targetPage)
{
    GHandle **current_page = pages[page];
    // printf("Clear %d\n", page_sizes[page]);
    for (int i = 0; i < page_sizes[page]; i++)
    {
        // printf("%d\n", i);
        gwinSetVisible(*(current_page[i]), gFalse);
    }
    GHandle **next_page = pages[targetPage];
    // printf("Target %d\n", page_sizes[targetPage]);
    for (int i = 0; i < page_sizes[targetPage]; i++)
    {
        // printf("%d\n", i);
        gwinSetVisible(*(next_page[i]), gTrue);
    }
    if (targetPage == 0)
    {
        gwinSetVisible(left_button_handle, gFalse);
        gwinSetVisible(right_button_handle, gTrue);
    }
    else if (targetPage >= max_pages - 1)
    {
        gwinSetVisible(left_button_handle, gTrue);
        gwinSetVisible(right_button_handle, gFalse);
    }
    else
    {
        gwinSetVisible(left_button_handle, gTrue);
        gwinSetVisible(right_button_handle, gTrue);
    }
    gwinRedrawDisplay(NULL, gFalse);
}
static void mouse_task(__unused void *params)
{
    geventListenerInit(&gl);
    gwinAttachListener(&gl);
    GEvent *pem;
    while(1)
    {
        //  printf("Get event task\n");
        pem = geventEventWait(&gl, TIME_INFINITE);
        if (pem != NULL)
        {
            switch (pem->type)
            {
                case GEVENT_GWIN_BUTTON:
                    if (((GEventGWinButton*)pem)->gwin == right_button_handle) {
                        
                        // printf("%lu\n", max_pages);
                        if (page < max_pages)
                        {
                            // printf("next page\n");
                            switch_page(page + 1);
                            page++;
                        }
                    }
                    else if (((GEventGWinButton*)pem)->gwin == left_button_handle) {
                        if (page > 0)
                        {
                            switch_page(page - 1);
                            page--;
                        }
                    }
                    break;
    
                default:
                    break;
            }
        }
        geventEventComplete(&gl);
    }
}

static void poll_data_task(__unused void *params)
{
    int scd41_interval = 5;
    do
    {
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        bool scd41_dataReady = false;
        scd41_interval--;
        if (scd41_interval <= 0)
        {
            if (!scd41_getDataReadyFlag(scd41, &scd41_dataReady))
            {
                printf("SCD41 fail to get data ready\n");
            }
            else if (scd41_dataReady)
            {
                scd41_data_ready = scd41_readMeasurement(scd41);
                if (scd41_data_ready)
                {
                    sgp40_voc_index = sgp40_measureVOCIndex(sgp40, scd41->temperature, scd41->humidity);
                    sgp40_data_ready = true;
                }
                else
                {
                    sgp40_data_ready = false;
                }
            }
            scd41_interval = 5;
        }
        if (bsec_sample_ready(bme688))
        {
            bme688_data_ready = bsec_read_data(bme688);
        }
        else
        {
            bme688_data_ready = false;
        }
        pm25_data_ready = pms25aqi_read(pms25, &pm25Data);
        xSemaphoreGive(data_mutex);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while (1);
}

static void init_data_widget(data_widget *widget, int x, int y, const char* label, int numDigits, bool init_visible)
{
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);
    wi.g.show = gFalse;
    wi.g.x = x;
    wi.g.y = y;
    wi.g.width =  gdispGetFontMetric(numberFont, gFontMaxWidth)*numDigits;
    wi.g.height = gdispGetFontMetric(numberFont, gFontHeight);
    widget->data = gwinLabelCreate(NULL, &wi);
    gwinSetFont(widget->data , numberFont);
    gwinLabelSetAttribute(widget->data , 0, NULL);
    gwinSetText(widget->data , "-", gTrue);
    gwinSetCustomDraw(widget->data , gwinLabelDrawJustifiedLeft, NULL);
    gwinSetVisible(widget->data , init_visible);

    wi.g.show = gFalse;
    wi.g.x = x;
    wi.g.y = y + gdispGetFontMetric(numberFont, gFontHeight);
    wi.g.height = gdispGetFontMetric(labelFont, gFontHeight);
    wi.text = label;
    wi.g.width =  gdispGetFontMetric(labelFont, gFontMaxWidth)*strlen(wi.text);
    widget->name = gwinLabelCreate(NULL, &wi);
    gwinSetFont(widget->name, labelFont);
    gwinLabelSetAttribute(widget->name, 0, NULL);
    gwinSetCustomDraw(widget->name, gwinLabelDrawJustifiedLeft, NULL);
    gwinSetVisible(widget->name, init_visible);
}

static void init_widgets()
{
    page = 0;
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);

    // Create our title
	font = gdispOpenFont("UI2");
    numberFont = gdispOpenFont("DejaVuSans20");
    labelFont = gdispOpenFont("DejaVuSans12");
	gwinSetDefaultFont(font);
	bHeight = gdispGetFontMetric(numberFont, gFontHeight);    
    // Apply the label parameters
    wi.g.show = gFalse;
    wi.g.y = 0;
    wi.g.x = 70;
    wi.g.width = swidth - 140;
    wi.g.height = bHeight;
    wi.text = "SCD41";
    // Create the actual label
    scd41_handle.name = gwinLabelCreate(NULL, &wi);
    gwinSetFont(scd41_handle.name, numberFont);
    gwinLabelSetAttribute(scd41_handle.name, 0, NULL);
    gwinSetCustomDraw(scd41_handle.name, gwinLabelDrawJustifiedCenter, NULL);
    gwinSetVisible(scd41_handle.name, gTrue);

    int y = bHeight;
    init_data_widget(&scd41_handle.co2, 0, y, "CO2 PPM", 4, gTrue);
    init_data_widget(&scd41_handle.temperature, swidth / 2 - 30, y, "Temp *C", 4, gTrue);
    init_data_widget(&scd41_handle.humidity, swidth - 70, y, "Humidity %", 4, gTrue);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight) + 5;

    wi.g.show = gFalse;
    wi.g.y = y;
    wi.g.x = 0;
    wi.g.width = swidth;
    wi.g.height = bHeight;
    wi.text = "BME668";
    // Create the actual label
    bme688_handle.name = gwinLabelCreate(NULL, &wi);
    gwinSetFont(bme688_handle.name, numberFont);
    gwinLabelSetAttribute(bme688_handle.name, 0, NULL);
    gwinSetCustomDraw(bme688_handle.name, gwinLabelDrawJustifiedCenter, NULL);
    gwinSetVisible(bme688_handle.name, gTrue);

    y+= bHeight;

    init_data_widget(&bme688_handle.voc_index, 0, y, "VOC Index", 6, gTrue);
    init_data_widget(&bme688_handle.temperature, swidth / 2 - 30, y, "Temp *C", 4, gTrue);
    init_data_widget(&bme688_handle.pressure, swidth - 80, y, "Press hPa", 6, gTrue);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight) + 5;

    init_data_widget(&bme688_handle.humidity,  0, y, "Humidity %", 4, gTrue);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight) + 5;

    wi.g.show = gFalse;
    wi.g.y = y;
    wi.g.x = 0;
    wi.g.width = swidth;
    wi.g.height = bHeight;
    wi.text = "SGP40";
    // Create the actual label
    sgp40_handle.name = gwinLabelCreate(NULL, &wi);
    gwinSetFont(sgp40_handle.name, numberFont);
    gwinLabelSetAttribute(sgp40_handle.name, 0, NULL);
    gwinSetCustomDraw(sgp40_handle.name, gwinLabelDrawJustifiedCenter, NULL);
    gwinSetVisible(sgp40_handle.name, gTrue);

    y+= bHeight;

    init_data_widget(&sgp40_handle.voc_index,  0, y, "VOC Index", 4, gTrue);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight) + 5;

    wi.g.show = gFalse;
    wi.g.y = y;
    wi.g.x = 0;
    wi.g.width = swidth;
    wi.g.height = bHeight;
    wi.text = "PM2.5";
    // Create the actual label
    pm25_handle.name = gwinLabelCreate(NULL, &wi);
    gwinSetFont(pm25_handle.name, numberFont);
    gwinLabelSetAttribute(pm25_handle.name, 0, NULL);
    gwinSetCustomDraw(pm25_handle.name, gwinLabelDrawJustifiedCenter, NULL);
    gwinSetVisible(pm25_handle.name, gTrue);

    y += bHeight;

    init_data_widget(&pm25_handle.pm25_standard, 0, y, "2.5 std", 4, gTrue);
    init_data_widget(&pm25_handle.pm10_standard, swidth / 2 - 30, y, "1.0 std", 4, gTrue);
    init_data_widget(&pm25_handle.pm100_standard, swidth - 70, y, "100 std", 4, gTrue);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight);

    init_data_widget(&pm25_handle.pm25_env, 0, y, "2.5 env", 4, gTrue);
    init_data_widget(&pm25_handle.pm10_env, swidth / 2 - 30, y, "1.0 env", 4, gTrue);
    init_data_widget(&pm25_handle.pm100_env, swidth - 70, y, "100 env", 4, gTrue);

    wi.g.x = swidth - 70;
    wi.g.y = 0;
    wi.g.width = 70;
    wi.g.height = bHeight;
    wi.g.show = gFalse;
    wi.text = "";
    right_button_handle = gwinButtonCreate(NULL, &wi);
    gwinSetCustomDraw(right_button_handle, gwinButtonDraw_ArrowRight, NULL);
    gwinSetVisible(right_button_handle, gTrue);

    wi.g.x = 0;
    wi.g.y = 0;
    wi.g.width = 70;
    wi.g.height = bHeight;
    wi.g.show = gFalse;
    wi.text = "";
    left_button_handle = gwinButtonCreate(NULL, &wi);
    gwinSetCustomDraw(left_button_handle, gwinButtonDraw_ArrowLeft, NULL);
    gwinSetVisible(left_button_handle, gFalse);

    wi.g.show = gFalse;
    wi.g.y = 0;
    wi.g.x = 70;
    wi.g.width = swidth-140;
    wi.g.height = bHeight;
    wi.text = "PM25";
    // Create the actual label
    pm25_handle.name_page_two = gwinLabelCreate(NULL, &wi);
    gwinSetFont(pm25_handle.name_page_two, numberFont);
    gwinLabelSetAttribute(pm25_handle.name_page_two, 0, NULL);
    gwinSetCustomDraw(pm25_handle.name_page_two, gwinLabelDrawJustifiedCenter, NULL);

    y = bHeight;

    init_data_widget(&pm25_handle.particles_03, 0, y, "3 um", 4, gFalse);
    init_data_widget(&pm25_handle.particles_05, swidth / 2 - 30, y, "5 um", 4, gFalse);
    init_data_widget(&pm25_handle.particles_10, swidth - 70, y, "10 um", 4, gFalse);

    y += bHeight + gdispGetFontMetric(labelFont, gFontHeight);

    init_data_widget(&pm25_handle.particles_25, 0, y, "25 um", 4, gFalse);
    init_data_widget(&pm25_handle.particles_50, swidth / 2 - 30, y, "50 um", 4, gFalse);
    init_data_widget(&pm25_handle.particles_100, swidth - 70, y, "100 um", 4, gFalse);

}

static void main_task(__unused void *params) 
{
    i2c_mutex = xSemaphoreCreateMutex();
    data_mutex = xSemaphoreCreateMutex();
    i2c_init(i2c1, 100*1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    pms25 = pms25aqi_init(i2c1, &i2c_mutex, false, 0, 0, 0);
    sgp40 = sgp40_init(i2c1, &i2c_mutex, 1.0, false, 0, 0, 0);
    bme688 = bsec_init(i2c1, &i2c_mutex, false, 0, 0, 0);
    scd41 = scd41_init(i2c1, &i2c_mutex, false, 0, 0, 0);

    memset(&pm25Data, 0, sizeof(pm25Data));
    sgp40_voc_index = 0;
    pm25_data_ready = false;
    bme688_data_ready = false;
    scd41_data_ready = false;
    sgp40_data_ready = false;
    gfxInit();
    // ginputCalibrateMouse(0);
    bool sgp40_ok = sgp40_begin(sgp40);
    bool bme688_ok = bsec_begin(bme688);
    bool scd41_ok = scd41_begin(scd41);
    // Get the display dimensions
	swidth = gdispGetWidth();
	sheight = gdispGetHeight();


    init_widgets();
    if (!sgp40_ok)
    {
        // gwinPrintg(scd41_handle, "Failed to initialize SGP40\n");
    }
    if(!bme688_ok)
    {
        // gwinPrintg(bme688_handle, "Failed to initialize BME688!\n");
    }
    if (!scd41_ok)
    {
        // gwinPrintg(scd41_handle, "Failed to initialize SCD40\n");
    }
    // scd41_wakeUp(scd41);
    // vTaskDelay(30 / portTICK_PERIOD_MS);
    if (!scd41_stopPeriodicMeasurement(scd41))
    {
        // gwinPrintg(scd41_handle, "SCD41 stop periodic measurement fail\n");
        printf("SCD41 stop periodic measurement fail\n");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // scd41_reinit(scd41);
    // vTaskDelay(30 / portTICK_PERIOD_MS);
    {
        
        uint16_t serial1, serial2, serial3;
        if (!scd41_getSerialNumber(scd41, &serial1, &serial2, &serial3))
        {
            // gwinPrintg(scd41_handle, "SCD41 fail to get serial number\n");
            printf("SCD41 fail to get serial number\n");
            scd41->init_ok = false;
        }
        else if (!scd41_startPeriodicMeasurement(scd41))
        {
            // gwinPrintg(scd41_handle, "SCD41 fail to begin periodic measurement\n");
            printf("SCD41 fail to begin periodic measurement\n");
            scd41->init_ok = false;
        }
        else
        {
            // gwinPrintg(scd41_handle, "SCD41 SerialNumber: %u, %u, %u\n", (unsigned int)serial1, (unsigned int)serial2, (unsigned int)serial3);
            printf("SCD41 SerialNumber: %u, %u, %u\n", (unsigned int)serial1, (unsigned int)serial2, (unsigned int)serial3);

        }
    }
    xTaskCreate(poll_data_task, "Poll Data", WORKER_TASK_STACK_SIZE, NULL, WORKER_TASK_PRIORITY, &poll_task);
    xTaskCreate(mouse_task, "Button Handler Task", WORKER_TASK_STACK_SIZE, NULL, WORKER_TASK_PRIORITY, &button_task_handle);
    while(1)
    {
        if (xSemaphoreTake(data_mutex, 0))
        {
            if (scd41_data_ready)
            {
                char buf[16];

                snprintf(buf, 16, "%d", (int)scd41->co2);
                gwinSetText(scd41_handle.co2.data, buf, gTrue);
                snprintf(buf, 16, "%0.2f", scd41->temperature);
                gwinSetText(scd41_handle.temperature.data, buf, gTrue);
                snprintf(buf, 16, "%0.2f", scd41->humidity);
                gwinSetText(scd41_handle.humidity.data, buf, gTrue);
            }
            if (bme688_data_ready)
            {
                char bme688_buf[16];
                snprintf(bme688_buf, 16, "%d", bme688->sensor->pressure);   
                gwinSetText(bme688_handle.pressure.data, bme688_buf, gTrue);
                snprintf(bme688_buf, 16, "%0.2f", bme688->temperature);
                gwinSetText(bme688_handle.temperature.data, bme688_buf, gTrue);
                snprintf(bme688_buf, 16, "%0.2f", bme688->humidity);
                gwinSetText(bme688_handle.humidity.data, bme688_buf, gTrue);
                snprintf(bme688_buf, 16, "%d", bme688->voc_index);
                gwinSetText(bme688_handle.voc_index.data, bme688_buf, gTrue);
            }
            if (sgp40_data_ready)
            {
                char sgp40_buf[8];
                snprintf(sgp40_buf, 16, "%d", sgp40_voc_index);   
                gwinSetText(sgp40_handle.voc_index.data, sgp40_buf, gTrue);
            }
            if (pm25_data_ready)
            {
                char pm25_buf[8];
                snprintf(pm25_buf, 8, "%d", pm25Data.pm25_standard);   
                gwinSetText(pm25_handle.pm25_standard.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.pm10_standard);   
                gwinSetText(pm25_handle.pm10_standard.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.pm100_standard);   
                gwinSetText(pm25_handle.pm100_standard.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.pm25_env);   
                gwinSetText(pm25_handle.pm25_env.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.pm10_env);   
                gwinSetText(pm25_handle.pm10_env.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.pm100_env);   
                gwinSetText(pm25_handle.pm100_env.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_03um);   
                gwinSetText(pm25_handle.particles_03.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_05um);   
                gwinSetText(pm25_handle.particles_05.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_10um);   
                gwinSetText(pm25_handle.particles_10.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_25um);   
                gwinSetText(pm25_handle.particles_25.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_50um);   
                gwinSetText(pm25_handle.particles_50.data, pm25_buf, gTrue);
                snprintf(pm25_buf, 8, "%d", pm25Data.particles_100um);   
                gwinSetText(pm25_handle.particles_100.data, pm25_buf, gTrue);
            }
            xSemaphoreGive(data_mutex);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    const char *rtos_name = "Air Quality Monitor";
    printf("Starting %s on core 0:\n", rtos_name);
    TaskHandle_t task;
    xTaskCreate(main_task, "MainThread", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    vTaskCoreAffinitySet(task, 0x01);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}
