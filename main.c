#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "headers/simulation.h"
#include "headers/mpu6050.h"

#define I2C_SDA_PIN  GPIO_NUM_21
#define I2C_SCL_PIN  GPIO_NUM_22

Simulation simulation;

int *currentLightsOn;

const int particleAmount = 35;
const int simulationWidth = 480;
const int simulationHeight = 480;

const int LED_COUNT = 64;

#define LED_GPIO    16

static led_strip_handle_t strip;
i2c_master_dev_handle_t mpu_dev;

void init_leds() {
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt_cfg, &strip));
}

void init_gyro() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    //i2c_master_dev_handle_t mpu_dev;
    ESP_ERROR_CHECK(mpu6050_init(i2c_bus, &mpu_dev));
}

void set_pixel(int led, uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(strip, led, r, g, b);
}

void refresh_leds(void) {
    led_strip_refresh(strip);
}

float getCurrentTime() {
    return (float)xTaskGetTickCount() / (float)configTICK_RATE_HZ;
}

float getDeltaTime(float lastTime)
{
    float now = getCurrentTime();
    float deltaTime = now - lastTime;
    return deltaTime;
}

static inline int* ledMatrixFit(Simulation *simulation)
{
    static int leds[64];
    const int ledGrid = 8;
    memset(leds, 0, sizeof(leds));
    for (int ly = 0; ly < ledGrid; ly++) {
        for (int lx = 0; lx < ledGrid; lx++) {
            if (simulation->slotFirstParticleIndexMatrix[ly * simulation->matrixWidth + lx] != -1) {
                leds[ly * ledGrid + lx] = 1;
            }
        }
    }
    return leds;
}

static inline void drawTiles(int *slotFirstParticleIndexMatrix, int matrixWidth, int matrixHeight)
{
    currentLightsOn = ledMatrixFit(&simulation);
    for(int i = 0; i < LED_COUNT; i++) {
        if(currentLightsOn[i] == 1) {
            set_pixel(i, 50, 50, 50);
        }
        else {
            set_pixel(i, 0, 0, 0);
        }
    }
    refresh_leds();
}

void app_main()
{
    init_leds();
    simulationInit(&simulation, simulationWidth, simulationHeight, particleAmount);
    init_gyro();
    
    float lastTime = getCurrentTime();
    while (1)
    {
        Mpu6050Data accel;
        if (mpu6050_read_accel(mpu_dev, &accel) == ESP_OK) {
            updateSimulation(&simulation, getDeltaTime(lastTime), accel.x, accel.y);
            //printf("accel x=%.3f y=%.3f z=%.3f\n", accel.x, accel.y, accel.z);
        } else {
            updateSimulation(&simulation, getDeltaTime(lastTime), 0.0f, 1.0f);
        }
        lastTime = getCurrentTime();
        drawTiles(getSlotFirstParticleIndexMatrix(&simulation), simulationWidth, simulationHeight);
    }
    simulationDestroy(&simulation);
}