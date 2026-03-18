#include "RPYSolver.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Fusion library includes (from repo/Fusion)
#include "FusionAhrs.h"
#include "FusionBias.h"
#include "FusionMath.h"
#include "FusionConvention.h"

// ESP32 timer includes
#include "esp_timer.h"

// ============================================================================
// Private State
// ============================================================================

static FusionAhrs ahrs;
static FusionBias bias;
static FusionAhrsSettings ahrs_settings;
// FIX: Using current_output matching the RPYOutput struct
static RPYOutput current_output = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
static bool initialized = false;
static float sample_period = 0.01f;  // 10 ms default (100 Hz)
static bool use_mag = false;
static uint32_t sample_rate_hz = 100;

// High-resolution timing for accurate delta time
static int64_t last_update_time_us = 0;
static float last_dt = 0.0f;

// Baseline gyro bias from startup calibration (deg/s)
static FusionVector startup_gyro_bias = {{0.0f, 0.0f, 0.0f}};
// Adaptive Z-axis bias estimator for no-mag yaw drift suppression (deg/s)
static float adaptive_gyro_bias_z = 0.0f;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Initialize RPY Solver
 */
void RPYSolver_Init(uint32_t sensor_sample_rate_hz, bool use_magnetometer, RPYMountOrientation orientation)
{
    if (sensor_sample_rate_hz == 0) sensor_sample_rate_hz = 100;
    
    sample_rate_hz = sensor_sample_rate_hz;
    sample_period = 1.0f / (float)sample_rate_hz;
    use_mag = use_magnetometer;
    (void)orientation;
    
    // Initialize gyroscope bias correction algorithm
    FusionBiasInitialise(&bias, sample_rate_hz);
    
    // Initialize AHRS with default settings
    FusionAhrsInitialise(&ahrs);
    
    // Get default settings and customize
    ahrs_settings = fusionAhrsDefaultSettings;
    ahrs_settings.convention = FusionConventionEnu;  // East-North-Up convention
    ahrs_settings.gain = 0.5f;                        // Standard gain
    ahrs_settings.accelerationRejection = 10.0f;     // 10 degrees threshold
    ahrs_settings.magneticRejection = 10.0f;         // 10 degrees threshold
    ahrs_settings.recoveryTriggerPeriod = (unsigned int)(5 * sample_rate_hz);  // 5 seconds
    ahrs_settings.gyroscopeRange = 2000.0f;          // ISM330DHCX max range is 2000 dps
    
    FusionAhrsSetSettings(&ahrs, &ahrs_settings);
    
    // Initialize high-resolution timer
    last_update_time_us = esp_timer_get_time();
    last_dt = sample_period;
    adaptive_gyro_bias_z = 0.0f;
    
    initialized = true;
}

/**
 * @brief Update RPY solver with new sensor data
 */
// FIX: Changed return type from RPYAngles to RPYOutput
RPYOutput RPYSolver_Update(const SensorData *sensor_data) 
{
    if (!initialized || sensor_data == NULL) {
        return current_output; // FIX: Changed current_rpy to current_output
    }
    
    // Calculate precise delta time using ESP32's high-resolution timer
    int64_t current_time_us = esp_timer_get_time();
    float delta_time = (float)(current_time_us - last_update_time_us) / 1000000.0f;
    last_update_time_us = current_time_us;
    
    // Clamp delta time to reasonable bounds to handle timing anomalies
    if (delta_time < 0.001f) delta_time = last_dt;  // Min 1ms
    if (delta_time > 0.1f) delta_time = last_dt;    // Max 100ms
    last_dt = delta_time;
    
    // Convert sensor data to Fusion format (gyroscope in degrees/s)
    FusionVector gyroscope = {
        .axis = {
            .x = sensor_data->gyro_x,
            .y = sensor_data->gyro_y,
            .z = sensor_data->gyro_z,
        }
    };

    // Apply startup calibration bias first.
    gyroscope.axis.x -= startup_gyro_bias.axis.x;
    gyroscope.axis.y -= startup_gyro_bias.axis.y;
    gyroscope.axis.z -= startup_gyro_bias.axis.z;

    // Stationary detection used for no-mag yaw drift suppression.
    const float accel_mag = sqrtf(sensor_data->accel_x * sensor_data->accel_x +
                                  sensor_data->accel_y * sensor_data->accel_y +
                                  sensor_data->accel_z * sensor_data->accel_z);
    const float gyro_norm = sqrtf(gyroscope.axis.x * gyroscope.axis.x +
                                  gyroscope.axis.y * gyroscope.axis.y +
                                  gyroscope.axis.z * gyroscope.axis.z);
    const bool stationary = (fabsf(accel_mag - 1.0f) < 0.05f) && (gyro_norm < 3.0f);

    // In 6-axis mode, adaptively estimate residual Z-bias during stationary windows.
    if (!use_mag && stationary) {
        adaptive_gyro_bias_z = (0.995f * adaptive_gyro_bias_z) + (0.005f * gyroscope.axis.z);
    }
    gyroscope.axis.z -= adaptive_gyro_bias_z;
    
    // Apply gyroscope bias correction (removes drift)
    gyroscope = FusionBiasUpdate(&bias, gyroscope);
    
    // Convert accelerometer (already in g units)
    FusionVector accelerometer = {
        .axis = {
            .x = sensor_data->accel_x,
            .y = sensor_data->accel_y,
            .z = sensor_data->accel_z,
        }
    };
    
    // Update AHRS
    if (use_mag && (sensor_data->mag_x != 0.0f || sensor_data->mag_y != 0.0f || sensor_data->mag_z != 0.0f)) {
        FusionVector magnetometer = {
            .axis = {
                .x = sensor_data->mag_x,
                .y = sensor_data->mag_y,
                .z = sensor_data->mag_z,
            }
        };
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);
    } else {
        // Use 6-axis version (gyro + accel only)
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);
    }
    
    // Get quaternion and convert to Euler angles (RPY)
    FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
    FusionEuler euler = FusionQuaternionToEuler(quaternion);
    
    // NEW: Extract pure linear acceleration (gravity removed)
    FusionVector linear_accel = FusionAhrsGetLinearAcceleration(&ahrs);
    
    // Populate the output struct
    current_output.roll = euler.angle.roll;
    current_output.pitch = euler.angle.pitch;
    current_output.yaw = euler.angle.yaw;
    
    // Output pure dynamic acceleration instead of raw sensor data
    current_output.accel_x = linear_accel.axis.x;
    current_output.accel_y = linear_accel.axis.y;
    current_output.accel_z = linear_accel.axis.z;
    
    return current_output;
}

/**
 * @brief Get current RPY angles
 */
// FIX: Renamed from RPYSolver_GetAngles to RPYSolver_GetOutput to match header
RPYOutput RPYSolver_GetOutput(void) 
{
    return current_output; // FIX: Changed current_rpy to current_output
}

/**
 * @brief Reset the solver to initial state
 */
void RPYSolver_Reset(void)
{
    if (initialized) {
        FusionAhrsReset(&ahrs);
        // FIX: Changed current_rpy to current_output
        current_output.roll = 0.0f;
        current_output.pitch = 0.0f;
        current_output.yaw = 0.0f;
        current_output.accel_x = 0.0f;
        current_output.accel_y = 0.0f;
        current_output.accel_z = 0.0f;
    }
}

/**
 * @brief Set algorithm gain (0.0 to 1.0+)
 */
void RPYSolver_SetGain(float gain)
{
    if (initialized) {
        ahrs_settings.gain = gain;
        FusionAhrsSetSettings(&ahrs, &ahrs_settings);
    }
}

/**
 * @brief Enable/disable acceleration rejection
 */
void RPYSolver_SetAccelerationRejection(float threshold)
{
    if (initialized) {
        ahrs_settings.accelerationRejection = threshold;
        FusionAhrsSetSettings(&ahrs, &ahrs_settings);
    }
}

/**
 * @brief Get internal status flags
 */
RPYSolver_Status RPYSolver_GetStatus(void)
{
    RPYSolver_Status status = {0};
    
    if (initialized) {
        FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs);
        status.initialising = flags.initialising;
        status.angular_rate_recovery = flags.angularRateRecovery;
        status.acceleration_recovery = flags.accelerationRecovery;
        status.magnetic_recovery = flags.magneticRecovery;
    }
    
    return status;
}

/**
 * @brief Get gyroscope bias status
 */
void RPYSolver_GetBiasStatus(void)
{
    if (!initialized) {
        return;
    }
}

/**
 * @brief Calibrate gyroscope at startup (device must be stationary)
 */
void RPYSolver_CalibrateGyro(void)
{
    if (!initialized) {
        return;
    }

    // Reset runtime bias estimator state; startup bias is provided separately.
    adaptive_gyro_bias_z = 0.0f;
    FusionBiasInitialise(&bias, sample_rate_hz);
}

void RPYSolver_SetGyroBias(float bias_x, float bias_y, float bias_z)
{
    startup_gyro_bias.axis.x = bias_x;
    startup_gyro_bias.axis.y = bias_y;
    startup_gyro_bias.axis.z = bias_z;
    adaptive_gyro_bias_z = 0.0f;
}