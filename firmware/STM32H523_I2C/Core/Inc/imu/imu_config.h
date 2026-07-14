#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#define PALM_IMU_COUNT 2U

#define PALM_IMU0_I2C_ADDR_7BIT 0x6AU
#define PALM_IMU1_I2C_ADDR_7BIT 0x6BU

#define PALM_IMU0_I2C_ADDR_HAL (PALM_IMU0_I2C_ADDR_7BIT << 1)
#define PALM_IMU1_I2C_ADDR_HAL (PALM_IMU1_I2C_ADDR_7BIT << 1)

#define PALM_IMU_FILTER_HZ 100U
#define PALM_IMU_FILTER_PERIOD_MS (1000U / PALM_IMU_FILTER_HZ)
#define PALM_IMU_FILTER_DT_S (1.0f / (float)PALM_IMU_FILTER_HZ)
#define PALM_IMU_WARMUP_SAMPLES 15U

#define PALM_LSM6DSOW_ACCEL_CTRL 0x48U
#define PALM_LSM6DSOW_GYRO_CTRL 0x4CU
#define PALM_LSM6DSOW_CTRL3_C_INIT 0x44U

#define PALM_LSM6DSOW_ACCEL_G_PER_LSB 0.000122f
#define PALM_LSM6DSOW_GYRO_DPS_PER_LSB 0.070f

/* Transmitted in frame byte 16; the palm remaps it to the host-visible node ID. */
#define PALM_LOCAL_NODE_INDEX 0U
#define PALM_MAX_LOCAL_NODE_INDEX 9U
#define PALM_PROTOCOL_HEADER 0xB6U
#define PALM_PROTOCOL_FRAME_SIZE 35U
#define PALM_RAW_PROTOCOL_HEADER 0xD6U
#define PALM_RAW_PROTOCOL_TYPE 0x01U
#define PALM_RAW_PROTOCOL_FRAME_SIZE 29U
#define PALM_FORCE_SCALE 100.0f
#define PALM_QUAT_SCALE 10000.0f
#define PALM_RAW_ACCEL_MG_SCALE 1000.0f
#define PALM_RAW_GYRO_DPS_X10_SCALE 10.0f

/* Drift tuning:
 * - Raise KP slightly if tilt correction is too weak while still.
 * - Raise KI only after bias calibration if slow tilt drift remains.
 * - Back KP down if motion causes noisy correction or overshoot.
 */

#define PALM_MAHONY_KP 0.8f
#define PALM_MAHONY_KI 0.01f

#define PALM_GYRO_BIAS_CALIBRATION_SAMPLES 300U
#define PALM_GYRO_BIAS_CALIBRATION_TIMEOUT_SAMPLES 400U
#define PALM_GYRO_BIAS_MAX_STILL_DPS 1.0f
#define PALM_ACCEL_NORM_STILL_TOL_G 0.10f

#define PALM_IMU0_AXIS_SIGN_X 1.0f
#define PALM_IMU0_AXIS_SIGN_Y 1.0f
#define PALM_IMU0_AXIS_SIGN_Z 1.0f

/* Default to the same sign correction used by the palm board. */
#define PALM_IMU1_AXIS_SIGN_X -1.0f
#define PALM_IMU1_AXIS_SIGN_Y -1.0f
#define PALM_IMU1_AXIS_SIGN_Z 1.0f

#define PALM_STATUS_IMU0_OK           (1U << 0)
#define PALM_STATUS_IMU1_OK           (1U << 1)
#define PALM_STATUS_FUSION_READY      (1U << 2)
#define PALM_STATUS_SINGLE_IMU_MODE   (1U << 3)
#define PALM_STATUS_USB_BUSY          (1U << 4)
#define PALM_STATUS_CALIBRATING       (1U << 5)
#define PALM_STATUS_FILTER_WARMUP     (1U << 6)

#if (PALM_LOCAL_NODE_INDEX > PALM_MAX_LOCAL_NODE_INDEX)
#error "PALM_LOCAL_NODE_INDEX must stay in the local sender range 0..9"
#endif

#endif /* IMU_CONFIG_H */
