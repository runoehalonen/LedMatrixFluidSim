#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#define MPU6050_I2C_ADDR     0x68
#define MPU6050_REG_PWR_MGMT 0x6B
#define MPU6050_REG_ACCEL    0x3B
#define MPU6050_GRAVITY_LSB  16384.0f  // ±2g default range: 16384 LSB/g

typedef struct {
    float x, y, z;  // acceleration in g
} Mpu6050Data;

static inline esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_dev)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
    if (err != ESP_OK) return err;

    // Clear sleep bit to wake the sensor
    uint8_t wake_cmd[] = { MPU6050_REG_PWR_MGMT, 0x00 };
    return i2c_master_transmit(*out_dev, wake_cmd, sizeof(wake_cmd), 100);
}

static inline esp_err_t mpu6050_read_accel(i2c_master_dev_handle_t dev, Mpu6050Data *out)
{
    uint8_t reg = MPU6050_REG_ACCEL;
    uint8_t buf[6];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, sizeof(buf), 100);
    if (err != ESP_OK) return err;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    out->x = ax / MPU6050_GRAVITY_LSB;
    out->y = ay / MPU6050_GRAVITY_LSB;
    out->z = az / MPU6050_GRAVITY_LSB;
    return ESP_OK;
}
