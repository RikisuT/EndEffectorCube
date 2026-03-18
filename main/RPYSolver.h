#ifndef RPY_SOLVER_H
#define RPY_SOLVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor mounting orientation for coordinate remapping
 */
typedef enum {
  RPY_MOUNT_STANDARD, // Default: X-forward, Y-left, Z-up
  RPY_MOUNT_90_X,     // Rotated 90° around X-axis
  RPY_MOUNT_90_Y,     // Rotated 90° around Y-axis
  RPY_MOUNT_90_Z,     // Rotated 90° around Z-axis
  RPY_MOUNT_180_X,    // Upside down
} RPYMountOrientation;

/**
 * @brief Output data with RPY + accelerometer
 */
typedef struct {
  float roll, pitch, yaw;          // RPY in degrees
  float accel_x, accel_y, accel_z; // Linear acceleration in g
} RPYOutput;

/**
 * @brief Sensor data structure
 */
typedef struct {
  float accel_x, accel_y, accel_z; // Accelerometer (g units)
  float gyro_x, gyro_y, gyro_z;    // Gyroscope (deg/s)
  float mag_x, mag_y, mag_z;       // Magnetometer (optional, can be zero)
} SensorData;

/**
 * @brief Initialize RPY Solver
 * @param sample_rate_hz Sample rate of sensor updates in Hz
 * @param use_magnetometer Set to true if magnetometer data available
 * @param mount_orientation How the IMU is mounted (for axis remapping)
 */
void RPYSolver_Init(uint32_t sample_rate_hz, bool use_magnetometer,
                    RPYMountOrientation mount_orientation);

/**
 * @brief Update RPY solver with new sensor data (FASTEST PATH - minimal
 * latency)
 * @param sensor_data Input sensor readings
 * @return RPY angles + accelerometer data
 */
RPYOutput RPYSolver_Update(const SensorData *sensor_data);

/**
 * @brief Get last computed output (non-blocking)
 * @return Current RPY angles + accelerometer
 */
RPYOutput RPYSolver_GetOutput(void);

/**
 * @brief Reset the solver to initial state
 */
void RPYSolver_Reset(void);

/**
 * @brief Set algorithm gain (0.0 to 1.0+)
 * @param gain Gain value (0.5 typical for most applications)
 */
void RPYSolver_SetGain(float gain);

/**
 * @brief Enable/disable acceleration rejection
 * @param threshold Threshold in degrees (0 to disable, 10 typical)
 */
void RPYSolver_SetAccelerationRejection(float threshold);

/**
 * @brief Get internal status flags
 */
typedef struct {
  bool initialising;
  bool angular_rate_recovery;
  bool acceleration_recovery;
  bool magnetic_recovery;
} RPYSolver_Status;

RPYSolver_Status RPYSolver_GetStatus(void);

/**
 * @brief Calibrate gyroscope bias (call at startup with device stationary)
 * Captures initial gyro readings to establish baseline for drift compensation
 * Blocks for ~1 second while collecting calibration samples
 */
void RPYSolver_CalibrateGyro(void);

/**
 * @brief Set gyroscope bias (deg/s) measured during startup calibration.
 * This baseline bias is applied before runtime bias estimation.
 */
void RPYSolver_SetGyroBias(float bias_x, float bias_y, float bias_z);

#ifdef __cplusplus
}
#endif

#endif // RPY_SOLVER_H
