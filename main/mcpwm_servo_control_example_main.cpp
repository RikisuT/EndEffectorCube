#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

extern "C" {
#include "VL53L4CD_api.h"
}

static const char *TAG = "SmartElex_ToF";

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_FREQ_HZ   400000
#define SENSOR_I2C_ADDR      0x29

extern "C" void app_main(void)
{
    // ----------------------------------------------------------------
    // 1. Create the I2C master bus
    // ----------------------------------------------------------------
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_cfg = {};   // zero-init clears all fields,
    bus_cfg.i2c_port          = I2C_NUM_0; // including intr_priority,
    bus_cfg.sda_io_num        = (gpio_num_t)I2C_MASTER_SDA_IO; // trans_queue_depth,
    bus_cfg.scl_io_num        = (gpio_num_t)I2C_MASTER_SCL_IO; // and allow_pd —
    bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;           // silences the
    bus_cfg.glitch_ignore_cnt = 7;                              // -Wmissing-field
    bus_cfg.flags.enable_internal_pullup = true;                // warnings too

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    // ----------------------------------------------------------------
    // 2. Allocate the device STRUCT (not the typedef pointer)
    //    VL53L5CX_Dev_t == VL53L5CX_Dev*  (it's already a pointer type)
    //    So declare VL53L5CX_Dev (the raw struct), then use its address
    //    as the VL53L5CX_Dev_t that every API call expects.
    // ----------------------------------------------------------------
    VL53L5CX_Dev sensor_dev = {};           // raw struct, zero-initialized
    VL53L5CX_Dev_t dev = &sensor_dev;      // dev is VL53L5CX_Dev* — correct type

    // ----------------------------------------------------------------
    // 3. Register sensor on the bus — this populates dev->handle
    //    which the platform layer uses for every I2C transaction
    // ----------------------------------------------------------------
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = SENSOR_I2C_ADDR;
    dev_cfg.scl_speed_hz    = I2C_MASTER_FREQ_HZ;
    // dev_cfg.scl_wait_us and .flags default to 0 — correct for standard mode

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->handle));
    dev->address = SENSOR_I2C_ADDR;   // informational, platform.c doesn't use it
    ESP_LOGI(TAG, "VL53L4CD registered on I2C bus at 0x%02X", SENSOR_I2C_ADDR);

    // ----------------------------------------------------------------
    // 4. Sensor init + configure + start
    // ----------------------------------------------------------------
    VL53L4CD_Error status;

    status = VL53L4CD_SensorInit(dev);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "SensorInit failed (status=%d) — check SDA=%d SCL=%d wiring",
                 status, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return;
    }
    ESP_LOGI(TAG, "Sensor initialized OK");

    status = VL53L4CD_SetRangeTiming(dev, 50, 0);  // 50ms budget, back-to-back
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "SetRangeTiming failed (status=%d)", status);
        return;
    }

    status = VL53L4CD_StartRanging(dev);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "StartRanging failed (status=%d)", status);
        return;
    }
    ESP_LOGI(TAG, "Ranging started — polling for measurements...");

    // ----------------------------------------------------------------
    // 5. Measurement loop
    // ----------------------------------------------------------------
    VL53L4CD_ResultsData_t results;
    uint8_t data_ready = 0;

    while (1) {
        VL53L4CD_CheckForDataReady(dev, &data_ready);

        if (data_ready) {
            status = VL53L4CD_GetResult(dev, &results);
            if (status == VL53L4CD_ERROR_NONE) {
                if (results.range_status == 0) {
                    ESP_LOGI(TAG, "Distance: %u mm | Signal: %lu kcps/SPAD",
                             results.distance_mm,
                             (unsigned long)results.signal_per_spad_kcps);
                } else {
                    // status codes: 1=sigma fail, 2=signal fail,
                    //               4=out of bounds, 7=wrap-around
                    ESP_LOGW(TAG, "Range status %u — measurement unreliable",
                             results.range_status);
                }
            } else {
                ESP_LOGE(TAG, "GetResult error (status=%d)", status);
            }
            VL53L4CD_ClearInterrupt(dev);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}