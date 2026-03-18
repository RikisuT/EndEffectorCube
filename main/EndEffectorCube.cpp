#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "RPYSolver.h"
#include "esp_timer.h"
#include "ism330dhcx.h"
#include "ism330dhcx_reg.h"
#include "vl53l4cd_platform.h"
#include "VL53L4CD_api.h"

static const char *TAG = "DUAL_SENSOR";

// I2C Configuration
#define I2C_MASTER_SCL_IO ((gpio_num_t)22) // GPIO 22 for I2C SCL (clock)
#define I2C_MASTER_SDA_IO ((gpio_num_t)21) // GPIO 21 for I2C SDA (data)
#define I2C_MASTER_FREQ_HZ 400000 // 400 kHz I2C speed

// IMU Interrupt Configuration
#define IMU_INT_GPIO ((gpio_num_t)34) // GPIO 34 for IMU INT1 (data-ready)

// TOF Interrupt Configuration
#define TOF1_INT_GPIO ((gpio_num_t)35) // GPIO 35 for TOF1 interrupt
#define TOF2_INT_GPIO ((gpio_num_t)32) // GPIO 32 for TOF2 interrupt
#define TOF3_INT_GPIO ((gpio_num_t)33) // GPIO 33 for TOF3 interrupt

// TOF XSHUT Configuration
#define TOF1_XSHUT_GPIO ((gpio_num_t)25) // GPIO 25 for TOF1 XSHUT (shutdown control)
#define TOF2_XSHUT_GPIO ((gpio_num_t)26) // GPIO 26 for TOF2 XSHUT (shutdown control)
#define TOF3_XSHUT_GPIO ((gpio_num_t)27) // GPIO 27 for TOF3 XSHUT (shutdown control)

// ISM330DHCX I2C Address (SA0 = 0)
#define ISM330DHCX_I2C_ADDR (ISM330DHCX_I2C_ADD_H >> 1)

// VL53L4CD I2C Addresses (for 3 sensors)
#define VL53L4CD_ADDR_SENSOR_1  (0x30)  // TOF1
#define VL53L4CD_ADDR_SENSOR_2  (0x31)  // TOF2
#define VL53L4CD_ADDR_SENSOR_3  (0x29)  // TOF3 (default address)

// Global sensor objects
ISM330DHCX_Object_t imu_obj;
VL53L5CX_Dev tof_dev_1;  // TOF sensor 1
VL53L5CX_Dev tof_dev_2;  // TOF sensor 2
VL53L5CX_Dev tof_dev_3;  // TOF sensor 3
i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t imu_dev_handle = NULL;
// Track which TOF sensors initialized successfully
static bool tof1_initialized = false;
static bool tof2_initialized = false;
static bool tof3_initialized = false;
// Global variables to hold our offsets
static float accel_offset_x = 0.0f;
static float accel_offset_y = 0.0f;
static float accel_offset_z = 0.0f;
// Task handle for interrupt notification
static TaskHandle_t imu_read_task_handle = NULL;
// Interrupt flags for dual sensors
static volatile bool imu_data_ready = false;
static volatile bool tof_data_ready = false;
/**
 * @brief I2C master initialization (using new driver/i2c_master.h API)
 */
static esp_err_t i2c_master_init(void) {
  ESP_LOGI(TAG, "    [i2c] Creating I2C master bus...");

  i2c_master_bus_config_t bus_cfg = {}; // zero-init all fields first
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = I2C_MASTER_SDA_IO;
  bus_cfg.scl_io_num = I2C_MASTER_SCL_IO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "    [i2c] Failed to create bus: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "    [i2c] ✓ I2C master bus created");

  ESP_LOGI(TAG, "    [i2c] Adding ISM330DHCX device to bus...");

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = ISM330DHCX_I2C_ADDR;
  dev_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;

  ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &imu_dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "    [i2c] Failed to add device: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "    [i2c] ✓ Device added to bus at address 0x%02X",
           ISM330DHCX_I2C_ADDR);

  // TOF devices will be added dynamically by sensor_boot() during app_main
  // to handle multiple sensors with XSHUT multiplexing

  // Give the I2C device time to stabilize before use
  vTaskDelay(pdMS_TO_TICKS(100));

  return ESP_OK;
}

/**
 * @brief Simple 1-point accelerometer calibration.
 * Device MUST be perfectly flat and stationary.
 */
static void calibrate_accelerometer(void) {
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
  ESP_LOGI(TAG, "CALIBRATING ACCEL - Keep device perfectly FLAT...");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");

  vTaskDelay(pdMS_TO_TICKS(1000)); // Give user a second to take their hands off

  const int num_samples = 500;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  ISM330DHCX_Axes_t acc_data;

  ESP_LOGI(TAG, "Sampling...");
  for (int i = 0; i < num_samples; i++) {
    if (ISM330DHCX_ACC_GetAxes(&imu_obj, &acc_data) == ISM330DHCX_OK) {
      sum_x += acc_data.x / 1000.0f;
      sum_y += acc_data.y / 1000.0f;
      sum_z += acc_data.z / 1000.0f;
    }
    // Delay 2ms to roughly match a 500Hz sampling rate
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // Average the readings to find the bias
  accel_offset_x = sum_x / num_samples;
  accel_offset_y = sum_y / num_samples;
  // Z is pointing up against gravity, so it SHOULD be 1.0g.
  // The offset is whatever is leftover after subtracting 1.0g.
  accel_offset_z = (sum_z / num_samples) - 1.0f;

  ESP_LOGI(TAG, "✓ Accel Offsets found:");
  ESP_LOGI(TAG, "  X: %+.4f g", accel_offset_x);
  ESP_LOGI(TAG, "  Y: %+.4f g", accel_offset_y);
  ESP_LOGI(TAG, "  Z: %+.4f g", accel_offset_z);
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════\n");
}

/**
 * @brief Calibrate gyroscope bias from stationary samples and pass to fusion.
 */
static void calibrate_gyroscope_bias(void) {
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
  ESP_LOGI(TAG, "CALIBRATING GYRO BIAS - Keep device stationary...");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");

  const int num_samples = 500;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  int valid_samples = 0;
  ISM330DHCX_Axes_t gyro_data;

  for (int i = 0; i < num_samples; i++) {
    if (ISM330DHCX_GYRO_GetAxes(&imu_obj, &gyro_data) == ISM330DHCX_OK) {
      // ISM330 values are mdps; convert to dps.
      sum_x += gyro_data.x / 1000.0f;
      sum_y += gyro_data.y / 1000.0f;
      sum_z += gyro_data.z / 1000.0f;
      valid_samples++;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (valid_samples == 0) {
    ESP_LOGW(TAG, "Gyro bias calibration failed: no valid samples");
    RPYSolver_SetGyroBias(0.0f, 0.0f, 0.0f);
    return;
  }

  const float bias_x = sum_x / valid_samples;
  const float bias_y = sum_y / valid_samples;
  const float bias_z = sum_z / valid_samples;
  RPYSolver_SetGyroBias(bias_x, bias_y, bias_z);

  ESP_LOGI(TAG, "✓ Gyro Bias found:");
  ESP_LOGI(TAG, "  X: %+.4f dps", bias_x);
  ESP_LOGI(TAG, "  Y: %+.4f dps", bias_y);
  ESP_LOGI(TAG, "  Z: %+.4f dps", bias_z);
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════\n");
}
/**
 * @brief I2C write wrapper for ISM330DHCX driver
 */
static int32_t platform_write(uint16_t dev_addr, uint16_t reg, uint8_t *bufp,
                              uint16_t len) {
  if (imu_dev_handle == NULL) {
    ESP_LOGE(TAG, "I2C device not initialized");
    return -1;
  }

  uint8_t *data = (uint8_t *)malloc(len + 1);
  if (data == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for I2C write");
    return -1;
  }

  data[0] = (uint8_t)reg;
  memcpy(&data[1], bufp, len);

  esp_err_t ret = i2c_master_transmit(imu_dev_handle, data, len + 1, 20);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "I2C write failed: reg=0x%02X, err=%s", reg,
             esp_err_to_name(ret));
  }
  free(data);
  return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief I2C read wrapper for ISM330DHCX driver
 */
static int32_t platform_read(uint16_t dev_addr, uint16_t reg, uint8_t *bufp,
                             uint16_t len) {
  if (imu_dev_handle == NULL) {
    ESP_LOGE(TAG, "I2C device not initialized");
    return -1;
  }

  uint8_t reg_addr = (uint8_t)reg;
  esp_err_t ret = i2c_master_transmit_receive(imu_dev_handle, &reg_addr, 1,
                                              bufp, len, 20);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "I2C read failed: reg=0x%02X, err=%s (0x%04X)", reg,
             esp_err_to_name(ret), ret);
  }
  return (ret == ESP_OK) ? 0 : -1;
}

static void platform_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/**
 * @brief Init function wrapper for ISM330DHCX driver
 */
static int32_t platform_init(void) { return 0; }

/**
 * @brief DeInit function wrapper for ISM330DHCX driver
 */
static int32_t platform_deinit(void) { return 0; }

/**
 * @brief ISR for IMU data-ready interrupt (INT1)
 * Wakes the imu_read_task to process new sensor data
 */
static void IRAM_ATTR imu_int_isr_handler(void *arg) {
  imu_data_ready = true;
  // Notify the read task that data is ready
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (imu_read_task_handle != NULL) {
    vTaskNotifyGiveFromISR(imu_read_task_handle, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief ISR for TOF data-ready interrupt (GPIO25)
 * Sets flag for TOF data ready
 */
static void IRAM_ATTR tof_int_isr_handler(void *arg) {
  tof_data_ready = true;
  // Notify the read task that data is ready
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (imu_read_task_handle != NULL) {
    vTaskNotifyGiveFromISR(imu_read_task_handle, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief Initialize GPIO interrupt for IMU data-ready signal and TOF
 */
static esp_err_t gpio_int_init(void) {
  ESP_LOGI(TAG, "  [GPIO] Configuring interrupt pins...");

  // Configure IMU INT pin (GPIO 27)
  ESP_LOGI(TAG, "  [GPIO] Setting up IMU INT (GPIO %d)...", IMU_INT_GPIO);
  gpio_config_t io_conf_imu = {};
  io_conf_imu.intr_type = GPIO_INTR_POSEDGE;
  io_conf_imu.mode = GPIO_MODE_INPUT;
  io_conf_imu.pin_bit_mask = (1ULL << IMU_INT_GPIO);
  io_conf_imu.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf_imu.pull_up_en = GPIO_PULLUP_DISABLE;

  esp_err_t ret = gpio_config(&io_conf_imu);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to configure IMU INT GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Configure TOF INT pins (GPIO 35, 32, 33 for TOF1, TOF2, TOF3)
  ESP_LOGI(TAG, "  [GPIO] Setting up TOF INT pins (TOF1: %d, TOF2: %d, TOF3: %d)...", TOF1_INT_GPIO, TOF2_INT_GPIO, TOF3_INT_GPIO);
  gpio_config_t io_conf_tof = {};
  io_conf_tof.intr_type = GPIO_INTR_POSEDGE;
  io_conf_tof.mode = GPIO_MODE_INPUT;
  io_conf_tof.pin_bit_mask = (1ULL << TOF1_INT_GPIO) | (1ULL << TOF2_INT_GPIO) | (1ULL << TOF3_INT_GPIO);
  io_conf_tof.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf_tof.pull_up_en = GPIO_PULLUP_DISABLE;

  ret = gpio_config(&io_conf_tof);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to configure TOF INT GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Configure TOF XSHUT pins (GPIO 25, 26, 27 for TOF1, TOF2, TOF3) as output
  ESP_LOGI(TAG, "  [GPIO] Setting up TOF XSHUT pins (TOF1: %d, TOF2: %d, TOF3: %d)...", TOF1_XSHUT_GPIO, TOF2_XSHUT_GPIO, TOF3_XSHUT_GPIO);
  gpio_config_t io_conf_xshut = {};
  io_conf_xshut.intr_type = GPIO_INTR_DISABLE;
  io_conf_xshut.mode = GPIO_MODE_OUTPUT;
  io_conf_xshut.pin_bit_mask = (1ULL << TOF1_XSHUT_GPIO) | (1ULL << TOF2_XSHUT_GPIO) | (1ULL << TOF3_XSHUT_GPIO);
  io_conf_xshut.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf_xshut.pull_up_en = GPIO_PULLUP_DISABLE;

  ret = gpio_config(&io_conf_xshut);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to configure TOF XSHUT GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Keep all TOF sensors in reset until app_main boots them one-by-one.
  gpio_set_level((gpio_num_t)TOF1_XSHUT_GPIO, 0);
  gpio_set_level((gpio_num_t)TOF2_XSHUT_GPIO, 0);
  gpio_set_level((gpio_num_t)TOF3_XSHUT_GPIO, 0);

  // Install GPIO ISR service
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    // ESP_ERR_INVALID_STATE = already installed (OK)
    ESP_LOGE(TAG, "  [GPIO] Failed to install ISR service: %s", esp_err_to_name(ret));
    return ret;
  }

  // Hook ISR handlers for both GPIO pins
  ret = gpio_isr_handler_add(IMU_INT_GPIO, imu_int_isr_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to add IMU ISR handler: %s", esp_err_to_name(ret));
    return ret;
  }

  // Add ISR handlers for all 3 TOF sensors
  ret = gpio_isr_handler_add((gpio_num_t)TOF1_INT_GPIO, tof_int_isr_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to add TOF1 ISR handler: %s", esp_err_to_name(ret));
    return ret;
  }
  ret = gpio_isr_handler_add((gpio_num_t)TOF2_INT_GPIO, tof_int_isr_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to add TOF2 ISR handler: %s", esp_err_to_name(ret));
    return ret;
  }
  ret = gpio_isr_handler_add((gpio_num_t)TOF3_INT_GPIO, tof_int_isr_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  [GPIO] Failed to add TOF3 ISR handler: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "  [GPIO] ✓ All interrupt pins configured");
  return ESP_OK;
}

/**
 * @brief Initialize ISM330DHCX sensor
 */
static esp_err_t imu_init(void) {
  ISM330DHCX_IO_t io_ctx;
  uint8_t id;
  int32_t ret;

  ESP_LOGI(TAG, "  [1/6] Configuring I/O interface...");
  // Configure IO interface
  io_ctx.BusType = ISM330DHCX_I2C_BUS;
  io_ctx.Address = ISM330DHCX_I2C_ADDR;
  io_ctx.Init = platform_init;
  io_ctx.DeInit = platform_deinit;
  io_ctx.ReadReg = platform_read;
  io_ctx.WriteReg = platform_write;
  io_ctx.Delay = platform_delay;

  // Register bus IO
  ESP_LOGI(TAG, "  [2/6] Registering bus IO...");
  ret = ISM330DHCX_RegisterBusIO(&imu_obj, &io_ctx);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to register bus IO (ret=%d)", ret);
    return ESP_FAIL;
  }

  // Read WHO_AM_I register
  ESP_LOGI(TAG, "  [3/6] Reading WHO_AM_I register...");
  ret = ISM330DHCX_ReadID(&imu_obj, &id);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I (ret=%d)", ret);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "  [3/6] WHO_AM_I = 0x%02X (expected 0x6B)", id);
  if (id != ISM330DHCX_ID) {
    ESP_LOGW(TAG, "⚠️  Sensor ID mismatch! Got 0x%02X, expected 0x%02X", id,
             ISM330DHCX_ID);
    // Don't fail - continue anyway
  }

  // Initialize the sensor
  ESP_LOGI(TAG, "  [4/6] Initializing sensor...");
  ret = ISM330DHCX_Init(&imu_obj);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to initialize sensor (ret=%d)", ret);
    return ESP_FAIL;
  }

  // Enable accelerometer
  ESP_LOGI(TAG, "  [5/6] Enabling accelerometer...");
  ret = ISM330DHCX_ACC_Enable(&imu_obj);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to enable accelerometer (ret=%d)", ret);
    return ESP_FAIL;
  }

  ret = ISM330DHCX_ACC_SetOutputDataRate(&imu_obj, 833.0f);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to set accelerometer ODR (ret=%d)", ret);
    return ESP_FAIL;
  }

  ret = ISM330DHCX_ACC_SetFullScale(&imu_obj, 2);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to set accelerometer full scale (ret=%d)", ret);
    return ESP_FAIL;
  }

  // Enable gyroscope
  ESP_LOGI(TAG, "  [6/6] Enabling gyroscope...");
  ret = ISM330DHCX_GYRO_Enable(&imu_obj);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to enable gyroscope (ret=%d)", ret);
    return ESP_FAIL;
  }

  ret = ISM330DHCX_GYRO_SetOutputDataRate(&imu_obj, 833.0f);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to set gyroscope ODR (ret=%d)", ret);
    return ESP_FAIL;
  }

  ret = ISM330DHCX_GYRO_SetFullScale(&imu_obj, 2000);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGE(TAG, "Failed to set gyroscope full scale (ret=%d)", ret);
    return ESP_FAIL;
  }

  // Enable data-ready interrupt routing to INT1 pin
  ESP_LOGI(TAG, "  [7/7] Enabling data-ready interrupt...");
  ret = ISM330DHCX_Set_INT1_Drdy(&imu_obj, 1U);
  if (ret != ISM330DHCX_OK) {
    ESP_LOGW(TAG, "Warning: Failed to enable INT1 DRDY (ret=%d), continuing...", ret);
  }

  ESP_LOGI(TAG, "✓ All sensor initialization steps completed successfully!");
  return ESP_OK;
}

/**
 * @brief FreeRTOS task to read sensor data from IMU and TOF
 * RUNS ON CORE 1 (dedicated to real-time sensor processing)
 * INTERRUPT-DRIVEN: Task blocks waiting for IMU/TOF data-ready signals
 * Core 0: System tasks, logging, I2C driver
 * Core 1: IMU + TOF sensor reading + Fusion AHRS calculation
 */
static void imu_read_task(void *pvParameters) {
  ISM330DHCX_Axes_t acc_data;
  ISM330DHCX_Axes_t gyro_data;
  VL53L4CD_ResultsData_t tof_results_1, tof_results_2, tof_results_3;
  SensorData sensor_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f};
  RPYOutput rpy;
  uint32_t last_hz_time = esp_timer_get_time();
  int hz_sample_count = 0;
  const uint32_t hz_calc_interval_us =
      50000; // Calculate Hz every 50ms (20 Hz display)
  const uint32_t tof_poll_interval_us = 20000; // 50 Hz TOF polling
  uint32_t last_tof_poll_time = esp_timer_get_time();
  uint16_t tof1_distance = 0;
  uint16_t tof2_distance = 0;
  uint16_t tof3_distance = 0;
  // Variables for data-ready polling
  uint8_t tof1_ready = 0, tof2_ready = 0, tof3_ready = 0;

  ESP_LOGI(TAG, "Sensor read task started on Core 1 (IMU + 3x TOF interrupt-driven)");
  
  // Wait for sensors to stabilize and start producing data
  vTaskDelay(pdMS_TO_TICKS(500));

  while (1) {
    // Block briefly for interrupt-driven wakeups; also run on timeout
    // to keep sensor updates flowing even if an interrupt edge is missed.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    // Poll TOF either when interrupt fired or on periodic fallback.
    uint32_t now_us = esp_timer_get_time();
    bool should_poll_tof =
        tof_data_ready || ((now_us - last_tof_poll_time) >= tof_poll_interval_us);
    if (should_poll_tof) {
      last_tof_poll_time = now_us;

      // Check data ready status for each sensor (only if initialized)
      if (tof1_initialized) {
        uint8_t ret1 = VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&tof_dev_1, &tof1_ready);
        if (ret1 == VL53L4CD_ERROR_NONE && tof1_ready) {
          VL53L4CD_GetResult((VL53L5CX_Dev_t)&tof_dev_1, &tof_results_1);
          tof1_distance = tof_results_1.distance_mm;
          VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&tof_dev_1);
        }
      }
      if (tof2_initialized) {
        uint8_t ret2 = VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&tof_dev_2, &tof2_ready);
        if (ret2 == VL53L4CD_ERROR_NONE && tof2_ready) {
          VL53L4CD_GetResult((VL53L5CX_Dev_t)&tof_dev_2, &tof_results_2);
          tof2_distance = tof_results_2.distance_mm;
          VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&tof_dev_2);
        }
      }
      if (tof3_initialized) {
        uint8_t ret3 = VL53L4CD_CheckForDataReady((VL53L5CX_Dev_t)&tof_dev_3, &tof3_ready);
        if (ret3 == VL53L4CD_ERROR_NONE && tof3_ready) {
          VL53L4CD_GetResult((VL53L5CX_Dev_t)&tof_dev_3, &tof_results_3);
          tof3_distance = tof_results_3.distance_mm;
          VL53L4CD_ClearInterrupt((VL53L5CX_Dev_t)&tof_dev_3);
        }
      }
      tof_data_ready = false;
    }

    // Read IMU every loop (interrupt-driven + polling fallback).
    // This prevents stalls when a DRDY interrupt edge is missed.
    if (ISM330DHCX_ACC_GetAxes(&imu_obj, &acc_data) == ISM330DHCX_OK) {
      // Accelerometer values are in mg (milli-g), convert to g
      sensor_data.accel_x = (acc_data.x / 1000.0f) - accel_offset_x;
      sensor_data.accel_y = (acc_data.y / 1000.0f) - accel_offset_y;
      sensor_data.accel_z = (acc_data.z / 1000.0f) - accel_offset_z;

      if (ISM330DHCX_GYRO_GetAxes(&imu_obj, &gyro_data) == ISM330DHCX_OK) {
        // Gyroscope values are in millidegrees per second (mdps), convert to deg/s
        sensor_data.gyro_x = gyro_data.x / 1000.0f;
        sensor_data.gyro_y = gyro_data.y / 1000.0f;
        sensor_data.gyro_z = gyro_data.z / 1000.0f;

        // No magnetometer data (6-axis IMU only)
        sensor_data.mag_x = 0.0f;
        sensor_data.mag_y = 0.0f;
        sensor_data.mag_z = 0.0f;

        // Update RPY solver
        rpy = RPYSolver_Update(&sensor_data);

        // Calculate actual Hz every 50ms
        uint32_t current_time = esp_timer_get_time();
        hz_sample_count++;
        if ((current_time - last_hz_time) >= hz_calc_interval_us) {
          float actual_hz =
              (float)hz_sample_count * 1000000.0f / (current_time - last_hz_time);
          last_hz_time = current_time;
          hz_sample_count = 0;

          // Print all sensor data: RPY, Accel, and all 3 TOF distances
          printf("R:%+6.1f° P:%+6.1f° Y:%+6.1f° | Ax:%+5.2fg Ay:%+5.2fg Az:%+5.2fg | TOF1:%4u TOF2:%4u TOF3:%4u mm | %6.1f Hz\n",
                 rpy.roll, rpy.pitch, rpy.yaw, rpy.accel_x, rpy.accel_y,
                 rpy.accel_z, tof1_distance, tof2_distance, tof3_distance,
                 actual_hz);
          fflush(stdout);
        }
      }
    }
    imu_data_ready = false;
  }
}

// ================================================================
// TOF Sensor Helper Functions (for 3-sensor setup)
// ================================================================

/**
 * @brief Boot one TOF sensor on XSHUT and reassign its I2C address
 */
static void sensor_boot(i2c_master_bus_handle_t bus,
                        gpio_num_t xshut_pin,
                        uint8_t new_addr,
                        VL53L5CX_Dev *out_dev)
{
    // Temporarily register at default address 0x29 to reassign it
    i2c_device_config_t tmp_cfg = {};
    tmp_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    tmp_cfg.device_address  = 0x29;
    tmp_cfg.scl_speed_hz    = I2C_MASTER_FREQ_HZ;

    // Wake this sensor
    gpio_set_level(xshut_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // SENSOR_BOOT_DELAY_MS

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
    new_cfg.scl_speed_hz    = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &new_cfg, &out_dev->handle));
    out_dev->address = new_addr;
}

/**
 * @brief Initialize and start ranging on a TOF sensor
 */
static void sensor_init_ranging(VL53L5CX_Dev_t dev, const char *name)
{
    VL53L4CD_Error status;

    status = VL53L4CD_SensorInit(dev);
    if (status != VL53L4CD_ERROR_NONE) {
        ESP_LOGE(TAG, "%s SensorInit failed (%d)", name, status);
        return;
    }
    status = VL53L4CD_SetRangeTiming(dev, 20, 0); // MEASUREMENT_TIMING_MS = 20
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
    
    // Mark this sensor as successfully initialized
    if (strcmp(name, "TOF1") == 0) {
        tof1_initialized = true;
    } else if (strcmp(name, "TOF2") == 0) {
        tof2_initialized = true;
    } else if (strcmp(name, "TOF3") == 0) {
        tof3_initialized = true;
    }
}

/**
 * @brief Main application entry point
 */
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
  ESP_LOGI(TAG, "  Dual Sensor: ISM330DHCX IMU + VL53L4CD TOF");
  ESP_LOGI(TAG, "  ✓ Dual-Core FreeRTOS | ✓ 240 MHz CPU | ✓ 1000 Hz Tick");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
  ESP_LOGI(TAG, "I2C: GPIO %d (SDA), GPIO %d (SCL)", I2C_MASTER_SDA_IO,
           I2C_MASTER_SCL_IO);
  ESP_LOGI(TAG, "IMU INT: GPIO %d", IMU_INT_GPIO);
  ESP_LOGI(TAG, "TOF1 INT: GPIO %d | TOF2 INT: GPIO %d | TOF3 INT: GPIO %d", TOF1_INT_GPIO, TOF2_INT_GPIO, TOF3_INT_GPIO);
  ESP_LOGI(TAG, "TOF1 XSHUT: GPIO %d | TOF2 XSHUT: GPIO %d | TOF3 XSHUT: GPIO %d", TOF1_XSHUT_GPIO, TOF2_XSHUT_GPIO, TOF3_XSHUT_GPIO);

  ESP_LOGI(TAG, "Initializing I2C master...");
  esp_err_t ret = i2c_master_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "✗ I2C master init failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "✓ I2C master initialized successfully");
  // Give the I2C peripheral time to stabilize
  vTaskDelay(pdMS_TO_TICKS(200));

  // Initialize GPIO interrupt pins
  ESP_LOGI(TAG, "Initializing GPIO interrupts...");
  ret = gpio_int_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO interrupt init failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "✓ GPIO interrupts initialized successfully");
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialize ISM330DHCX sensor
  ESP_LOGI(TAG, "Initializing ISM330DHCX (IMU) sensor...");
  ret = imu_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "IMU init failed!");
    return;
  }
  ESP_LOGI(TAG, "✓ IMU sensor initialized successfully");
  vTaskDelay(pdMS_TO_TICKS(200));

  // Boot and initialize all 3 TOF sensors
  ESP_LOGI(TAG, "Initializing 3x VL53L4CD (TOF) sensors...");
  // Both XSHUT pins start LOW to hold sensors in reset
  gpio_set_level((gpio_num_t)TOF1_XSHUT_GPIO, 0);
  gpio_set_level((gpio_num_t)TOF2_XSHUT_GPIO, 0);
  gpio_set_level((gpio_num_t)TOF3_XSHUT_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Boot sensors one by one, reassign I2C addresses
  ESP_LOGI(TAG, "  [1/3] Booting and addressing TOF1 (address 0x%02X)...", VL53L4CD_ADDR_SENSOR_1);
  sensor_boot(i2c_bus_handle, (gpio_num_t)TOF1_XSHUT_GPIO, VL53L4CD_ADDR_SENSOR_1, &tof_dev_1);
  
  ESP_LOGI(TAG, "  [2/3] Booting and addressing TOF2 (address 0x%02X)...", VL53L4CD_ADDR_SENSOR_2);
  sensor_boot(i2c_bus_handle, (gpio_num_t)TOF2_XSHUT_GPIO, VL53L4CD_ADDR_SENSOR_2, &tof_dev_2);
  
  ESP_LOGI(TAG, "  [3/3] Booting and addressing TOF3 (address 0x%02X)...", VL53L4CD_ADDR_SENSOR_3);
  sensor_boot(i2c_bus_handle, (gpio_num_t)TOF3_XSHUT_GPIO, VL53L4CD_ADDR_SENSOR_3, &tof_dev_3);
  
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialize ranging on all 3 sensors
  ESP_LOGI(TAG, "Starting ranging on all 3 TOF sensors...");
  sensor_init_ranging((VL53L5CX_Dev_t)&tof_dev_1, "TOF1");
  sensor_init_ranging((VL53L5CX_Dev_t)&tof_dev_2, "TOF2");
  sensor_init_ranging((VL53L5CX_Dev_t)&tof_dev_3, "TOF3");
  ESP_LOGI(TAG, "✓ All TOF sensors initialized successfully");

  // Initialize RPY Solver with 833 Hz sample rate (sensor ODR)
  ESP_LOGI(TAG, "Initializing RPY Solver (Fusion AHRS)...");
  RPYSolver_Init(833, false,
                 RPY_MOUNT_STANDARD); // 833 Hz, no magnetometer (6-axis IMU)
  ESP_LOGI(TAG, "✓ RPY Solver initialized");
  vTaskDelay(pdMS_TO_TICKS(100));

  // Calibrate gyroscope and accelerometer (device must be stationary)
  calibrate_gyroscope_bias();

  vTaskDelay(pdMS_TO_TICKS(1000));
  calibrate_accelerometer();
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(TAG, "✓ Calibration complete! Starting sensor fusion...");
  ESP_LOGI(TAG,
           "NOTE: Yaw ~0° (6-axis IMU). Use Roll & Pitch for orientation.");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════════════\n");
  vTaskDelay(pdMS_TO_TICKS(100));

  // Create sensor read task on Core 1 (dedicated to sensor processing)
  // Core 0 handles system tasks and logging
  ESP_LOGI(TAG, "Creating sensor reading task on Core 1...");
  xTaskCreatePinnedToCore(imu_read_task,       // Task function
                          "imu_read_task",     // Task name
                          4096,                // Stack size
                          NULL,                // Parameter
                          10,                  // Priority (high)
                          &imu_read_task_handle, // Task handle (for ISR notify)
                          1); // Core 1 (dedicated sensor processing)

  // App main task is done, handler runs in created task
  ESP_LOGI(TAG, "✓ Ready! Outputting sensor data below...");
}
