#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

extern "C" {
#include "VL53L4CD_api.h"
}

static const char *TAG = "EE_Position";

// --- I2C ---
#define I2C_SCL_IO          22
#define I2C_SDA_IO          21
#define I2C_FREQ_HZ         400000

// --- XSHUT pins (wire these from sensor board to ESP32 GPIO) ---
#define XSHUT_X             18
#define XSHUT_Y             19
#define XSHUT_Z             23

// --- Reassigned I2C addresses ---
#define ADDR_SENSOR_X       0x30
#define ADDR_SENSOR_Y       0x31
#define ADDR_SENSOR_Z       0x29   // last one stays at default

// --- Wall offsets: distance from each sensor's zero to the wall (mm) ---
// Measure these once and set them. If sensor is flush with wall at mount
// point, this equals the total workspace dimension on that axis.
#define WALL_X_MM           500
#define WALL_Y_MM           500
#define WALL_Z_MM           500

// --- Timing constants (milliseconds) ---
#define SENSOR_BOOT_DELAY_MS    10   // Datasheet: 1.2ms startup + margin
#define MEASUREMENT_TIMING_MS   20   // ~50Hz measurement rate
#define POLL_INTERVAL_MS        5    // Task polling interval

// ----------------------------------------------------------------
// Helper: bring up one sensor on XSHUT, assign it a new I2C address
// ----------------------------------------------------------------
static void sensor_boot(i2c_master_bus_handle_t bus,
                        gpio_num_t xshut_pin,
                        uint8_t new_addr,
                        VL53L5CX_Dev *out_dev)
{
    // Temporarily register at default address 0x29 to reassign it
    i2c_device_config_t tmp_cfg = {};
    tmp_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    tmp_cfg.device_address  = 0x29;
    tmp_cfg.scl_speed_hz    = I2C_FREQ_HZ;

    // Wake this sensor
    gpio_set_level(xshut_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_BOOT_DELAY_MS));

    i2c_master_dev_handle_t tmp_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &tmp_cfg, &tmp_handle));

    // Reassign address — VL53L4CD_SetI2CAddress writes to register 0x0001
    out_dev->handle  = tmp_handle;
    out_dev->address = 0x29;
    VL53L4CD_SetI2CAddress((VL53L5CX_Dev_t)out_dev, new_addr << 1); // API takes 8-bit shifted addr

    // Done with temp handle — remove and re-add at new address
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(tmp_handle));

    i2c_device_config_t new_cfg = {};
    new_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    new_cfg.device_address  = new_addr;
    new_cfg.scl_speed_hz    = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &new_cfg, &out_dev->handle));
    out_dev->address = new_addr;
}

// ----------------------------------------------------------------
// Helper: init + start ranging on one sensor
// ----------------------------------------------------------------
/**
 * Initialize and start ranging on a sensor.
 * @param dev Sensor device handle
 * @param name Axis name for logging ("X", "Y", or "Z")
 */
static void sensor_init_ranging(VL53L5CX_Dev_t dev, const char *name)
{
    VL53L4CD_Error status;

    status = VL53L4CD_SensorInit(dev);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "%s SensorInit failed (%d)", name, status);
        return;
    }
    status = VL53L4CD_SetRangeTiming(dev, MEASUREMENT_TIMING_MS, 0);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "%s SetRangeTiming failed (%d)", name, status);
        return;
    }
    status = VL53L4CD_StartRanging(dev);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "%s StartRanging failed (%d)", name, status);
        return;
    }
    ESP_LOGI(TAG, "%s ready at 0x%02X", name, dev->address);
}

extern "C" void app_main(void)
{
    // ----------------------------------------------------------------
    // 1. Configure XSHUT pins — pull all LOW to hold sensors in reset
    // ----------------------------------------------------------------
    gpio_config_t xshut_cfg = {};
    xshut_cfg.pin_bit_mask = (1ULL << XSHUT_X) |
                             (1ULL << XSHUT_Y) |
                             (1ULL << XSHUT_Z);
    xshut_cfg.mode         = GPIO_MODE_OUTPUT;
    xshut_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    xshut_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    xshut_cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&xshut_cfg));

    gpio_set_level((gpio_num_t)XSHUT_X, 0);
    gpio_set_level((gpio_num_t)XSHUT_Y, 0);
    gpio_set_level((gpio_num_t)XSHUT_Z, 0);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_BOOT_DELAY_MS));

    // ----------------------------------------------------------------
    // 2. I2C bus
    // ----------------------------------------------------------------
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port          = I2C_NUM_0;
    bus_cfg.sda_io_num        = (gpio_num_t)I2C_SDA_IO;
    bus_cfg.scl_io_num        = (gpio_num_t)I2C_SCL_IO;
    bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_LOGI(TAG, "I2C bus ready");

    // ----------------------------------------------------------------
    // 3. Boot sensors one at a time, reassign addresses
    // ----------------------------------------------------------------
    VL53L5CX_Dev dev_x = {}, dev_y = {}, dev_z = {};

    sensor_boot(bus, (gpio_num_t)XSHUT_X, ADDR_SENSOR_X, &dev_x);
    sensor_boot(bus, (gpio_num_t)XSHUT_Y, ADDR_SENSOR_Y, &dev_y);
    // Sensor Z: wake at default address (0x29)
    gpio_set_level((gpio_num_t)XSHUT_Z, 1);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_BOOT_DELAY_MS));
    i2c_device_config_t z_cfg = {};
    z_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    z_cfg.device_address  = ADDR_SENSOR_Z;
    z_cfg.scl_speed_hz    = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &z_cfg, &dev_z.handle));
    dev_z.address = ADDR_SENSOR_Z;

    // ----------------------------------------------------------------
    // 4. Init + start ranging on all three
    // ----------------------------------------------------------------
    sensor_init_ranging((VL53L5CX_Dev_t)&dev_x, "X");
    sensor_init_ranging((VL53L5CX_Dev_t)&dev_y, "Y");
    sensor_init_ranging((VL53L5CX_Dev_t)&dev_z, "Z");

    // ----------------------------------------------------------------
    // 5. Position loop
    // ----------------------------------------------------------------
    VL53L4CD_ResultsData_t result_x, result_y, result_z;
    uint8_t data_ready_x, data_ready_y, data_ready_z;

    while (1) {
        VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&dev_x, &data_ready_x);
        VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&dev_y, &data_ready_y);
        VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&dev_z, &data_ready_z);

        if (data_ready_x && data_ready_y && data_ready_z) {
            VL53L4CD_GetResult((VL53L5CX_Dev_t)&dev_x, &result_x);
            VL53L4CD_GetResult((VL53L5CX_Dev_t)&dev_y, &result_y);
            VL53L4CD_GetResult((VL53L5CX_Dev_t)&dev_z, &result_z);

            VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&dev_x);
            VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&dev_y);
            VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&dev_z);

            // Only log if all readings are valid
            if (result_x.range_status == 0 &&
                result_y.range_status == 0 &&
                result_z.range_status == 0) {

                int32_t position_x = WALL_X_MM - result_x.distance_mm;
                int32_t position_y = WALL_Y_MM - result_y.distance_mm;
                int32_t position_z = WALL_Z_MM - result_z.distance_mm;

                ESP_LOGI(TAG, "POS  X: %4ld mm  Y: %4ld mm  Z: %4ld mm",
                         position_x, position_y, position_z);
            } else {
                ESP_LOGW(TAG, "Unreliable — status X:%u Y:%u Z:%u",
                         result_x.range_status,
                         result_y.range_status,
                         result_z.range_status);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}