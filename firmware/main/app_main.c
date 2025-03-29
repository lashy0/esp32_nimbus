#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "bme680.h"
#include "bme68x_defs.h"

static const char *TAG = "app_main";

#define BME680_I2C_PORT         I2C_NUM_0
#define BME680_I2C_SDA_PIN      8
#define BME680_I2C_SCL_PIN      9

#define BME680_I2C_ADDR         BME68X_I2C_ADDR_HIGH 
#define BME680_CLK_SPEED        400000

void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = BME680_I2C_PORT,
        .sda_io_num = BME680_I2C_SDA_PIN,
        .scl_io_num = BME680_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_handle));

    ESP_LOGI(TAG, "Initialize BME680 sensor...");
    bme680_handle_t bme_sensor = NULL;

    esp_err_t ret = bme680_init(
        i2c_bus_handle,
        BME680_I2C_ADDR,
        BME680_CLK_SPEED,
        &bme_sensor
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME680 initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "BME680 instance initialized successfully.");
    vTaskDelay(pdMS_TO_TICKS(1000));

    bme680_data_t sensor_data;

    while (1) {
        ret = bme680_read_forced(bme_sensor, &sensor_data);

              
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Data: T=%.2fC H=%.2f%% P=%.0fPa",
                    sensor_data.temperature, sensor_data.humidity, sensor_data.pressure);

            // Проверяем статус газа перед выводом
            if ((sensor_data.status & BME68X_GASM_VALID_MSK) && (sensor_data.status & BME68X_HEAT_STAB_MSK)) {
                ESP_LOGI(TAG, "Gas Resistance: %lu Ohm (Stable)", sensor_data.gas_resistance);
            } else if (sensor_data.status & BME68X_GASM_VALID_MSK) {
                ESP_LOGW(TAG, "Gas Resistance: %lu Ohm (Heater Unstable)", sensor_data.gas_resistance);
            } else {
                ESP_LOGW(TAG, "Gas measurement not valid!");
            }
            ESP_LOGD(TAG,"Raw Status: 0x%02X", sensor_data.status);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
