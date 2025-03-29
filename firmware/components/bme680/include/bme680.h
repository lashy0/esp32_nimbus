#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "bme68x_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bme680_dev_s *bme680_handle_t;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    uint8_t status;
} bme680_data_t;

esp_err_t bme680_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_addr,
    uint32_t i2c_clk_hz,
    bme680_handle_t *out_handle
);

esp_err_t bme680_deinit(bme680_handle_t handle);

esp_err_t bme680_read_forced(bme680_handle_t handle, bme680_data_t *data);

#ifdef __cplusplus
}
#endif