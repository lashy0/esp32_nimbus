#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include "bme680.h"
#include "bme68x.h"

#include "esp_log.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

static const char *TAG = "bme680";

#define BME680_I2C_RW_TIMEOUT_MS 100

// Внутренняя структура экземпляра
struct bme680_dev_s {
    i2c_master_dev_handle_t dev_handle;
    struct bme68x_dev       bme_api_dev;
    struct bme68x_conf      current_conf;
    struct bme68x_heatr_conf current_heatr_conf;
    TickType_t              i2c_timeout_ticks;
    bool                    initialized;
    bool                    gas_measurement_enabled;
};

// Коллбеки для Bosch API (без изменений)

static BME68X_INTF_RET_TYPE bme68x_i2c_read_cb(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    struct bme680_dev_s *dev = (struct bme680_dev_s *)intf_ptr;
    if (!dev || !dev->dev_handle) return BME68X_E_NULL_PTR;
    if (len == 0) return BME68X_OK;
    esp_err_t err = i2c_master_transmit_receive(dev->dev_handle, &reg_addr, 1, reg_data, len, dev->i2c_timeout_ticks);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static BME68X_INTF_RET_TYPE bme68x_i2c_write_cb(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    struct bme680_dev_s *dev = (struct bme680_dev_s *)intf_ptr;
    if (!dev || !dev->dev_handle) return BME68X_E_NULL_PTR;
    esp_err_t err = ESP_FAIL;
    int write_size = len + 1;
    uint8_t *write_buf = (uint8_t *)alloca(write_size);
    if (!write_buf) return BME68X_E_NULL_PTR;
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, reg_data, len);
    err = i2c_master_transmit(dev->dev_handle, write_buf, write_size, dev->i2c_timeout_ticks);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void bme68x_delay_us_cb(uint32_t period_us, void *intf_ptr) {
    uint32_t ms = (period_us + 999) / 1000;
    if (ms > 0) vTaskDelay(pdMS_TO_TICKS(ms));
    else if (period_us > 0) vTaskDelay(1);
}

//

esp_err_t bme680_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_addr,
    uint32_t i2c_clk_hz,
    bme680_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(bus_handle && out_handle, ESP_ERR_INVALID_ARG, TAG, "Bus handle and out_handle must not be NULL");
    ESP_RETURN_ON_FALSE(i2c_addr == BME68X_I2C_ADDR_LOW || i2c_addr == BME68X_I2C_ADDR_HIGH,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid BME680 I2C address: 0x%02X", i2c_addr);
    ESP_RETURN_ON_FALSE(i2c_clk_hz > 0, ESP_ERR_INVALID_ARG, TAG, "I2C clock speed must be positive");

    esp_err_t ret = ESP_OK;
    int8_t bme_rslt = BME68X_OK;
    struct bme680_dev_s *dev = NULL;

    dev = (struct bme680_dev_s *)calloc(1, sizeof(struct bme680_dev_s));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for BME680 handle");

    dev->i2c_timeout_ticks = pdMS_TO_TICKS(BME680_I2C_RW_TIMEOUT_MS);
    dev->gas_measurement_enabled = true; // По умолчанию газ включен в тестовой версии драйвера

    // Добавляем устройство
    i2c_device_config_t i2c_dev_conf = { .device_address = i2c_addr, .scl_speed_hz = i2c_clk_hz };
    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &dev->dev_handle);
    ESP_GOTO_ON_ERROR(ret, cleanup_mem, TAG, "Adding BME680 device (0x%02X) failed: %s", i2c_addr, esp_err_to_name(ret));
    ESP_LOGD(TAG, "BME680 device (0x%02X) added to I2C bus.", i2c_addr);

    // Инициализируем структуру Bosch API
    dev->bme_api_dev.intf = BME68X_I2C_INTF;
    dev->bme_api_dev.read = bme68x_i2c_read_cb;
    dev->bme_api_dev.write = bme68x_i2c_write_cb;
    dev->bme_api_dev.delay_us = bme68x_delay_us_cb;
    dev->bme_api_dev.amb_temp = 25;
    dev->bme_api_dev.intf_ptr = dev;

    bme_rslt = bme68x_init(&dev->bme_api_dev);
    if (bme_rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x API init failed: %d", bme_rslt);
        ret = ESP_FAIL;
        goto cleanup_device;
    }
    ESP_LOGD(TAG, "BME68x API initialized. Chip ID: 0x%X", dev->bme_api_dev.chip_id);

    // Применяем настройки по умолчанию
    dev->current_conf.os_hum = BME68X_OS_2X;
    dev->current_conf.os_pres = BME68X_OS_4X;
    dev->current_conf.os_temp = BME68X_OS_8X;
    dev->current_conf.filter = BME68X_FILTER_SIZE_3;
    dev->current_conf.odr = BME68X_ODR_NONE;

    bme_rslt = bme68x_set_conf(&dev->current_conf, &dev->bme_api_dev);
    if (bme_rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set default sensor config: %d", bme_rslt);
        ret = ESP_FAIL;
        goto cleanup_device;
    }
    ESP_LOGD(TAG, "Default sensor T/P/H/Filter configuration applied.");

    // Применяем настройки нагревателя по умолчанию
    dev->current_heatr_conf.enable = BME68X_ENABLE; // Включен по умолчанию
    dev->current_heatr_conf.heatr_temp = 320;       // 320°C
    dev->current_heatr_conf.heatr_dur = 150;        // 150 мс

    bme_rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &dev->current_heatr_conf, &dev->bme_api_dev);
    if (bme_rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set default heater config: %d", bme_rslt);
        ret = ESP_FAIL;
        goto cleanup_device;
    }
    ESP_LOGD(TAG, "Default heater (Gas) configuration applied.");

    dev->initialized = true;
    *out_handle = (bme680_handle_t)dev;
    ESP_LOGI(TAG, "BME680 instance (0x%02X) initialized successfully!", i2c_addr);
    return ESP_OK;

cleanup_device:
    if (dev && dev->dev_handle) {
        i2c_master_bus_rm_device(dev->dev_handle);
    }
cleanup_mem:
    free(dev);
    *out_handle = NULL;
    return ret;
}

esp_err_t bme680_read_forced(bme680_handle_t handle, bme680_data_t *data) {
    struct bme680_dev_s *dev = (struct bme680_dev_s *)handle;
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "BME680 handle is invalid or not initialized");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Data pointer cannot be NULL");

    int8_t bme_rslt = BME68X_OK;
    uint32_t meas_delay_us = 0;
    uint8_t n_fields = 0;
    struct bme68x_data api_data;

    bme_rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev->bme_api_dev);
    ESP_RETURN_ON_FALSE(bme_rslt == BME68X_OK, ESP_FAIL, TAG, "Failed to set Forced Mode: %d", bme_rslt);

    meas_delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &dev->current_conf, &dev->bme_api_dev);
    ESP_LOGD(TAG, "Calculated measurement delay: %lu us", meas_delay_us);
    dev->bme_api_dev.delay_us(meas_delay_us, dev);

    bme_rslt = bme68x_get_data(BME68X_FORCED_MODE, &api_data, &n_fields, &dev->bme_api_dev);
     if (bme_rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to get data from sensor: %d", bme_rslt);
        return (bme_rslt == BME68X_E_COM_FAIL) ? ESP_ERR_TIMEOUT : ESP_FAIL;
    }

    if (n_fields > 0) {
        data->temperature = api_data.temperature;
        data->humidity = api_data.humidity;
        data->pressure = api_data.pressure;
        data->gas_resistance = api_data.gas_resistance;
        data->status = api_data.status;

        ESP_LOGD(TAG, "Data read: T=%.2f C, H=%.2f %%RH, P=%.0f Pa, G=%lu Ohm, Status=0x%X",
                 data->temperature, data->humidity, data->pressure, data->gas_resistance, data->status);

        if (!(api_data.status & BME68X_NEW_DATA_MSK)) {
             ESP_LOGW(TAG, "Sensor data status indicates no new data (status: 0x%X)", api_data.status);
             return ESP_ERR_INVALID_STATE;
        }
        if (dev->gas_measurement_enabled) { // Проверяем только если газ был включен при init
            if (!(api_data.status & BME68X_GASM_VALID_MSK)) ESP_LOGW(TAG, "Gas measurement not valid (status: 0x%X)", api_data.status);
            if (!(api_data.status & BME68X_HEAT_STAB_MSK)) ESP_LOGW(TAG, "Heater temp not stable (status: 0x%X)", api_data.status);
        }
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "No data fields received from sensor after measurement.");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t bme680_deinit(bme680_handle_t handle) {
    struct bme680_dev_s *dev = (struct bme680_dev_s *)handle;
    if (!dev) return ESP_OK;

    esp_err_t ret = ESP_OK;
    if (dev->dev_handle) {
        ret = i2c_master_bus_rm_device(dev->dev_handle);
        if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to remove BME680 device: %s", esp_err_to_name(ret));
        else ESP_LOGD(TAG, "BME680 device removed from bus.");
    }
    free(dev);
    ESP_LOGI(TAG, "BME680 instance deinitialized.");
    return ret;
}