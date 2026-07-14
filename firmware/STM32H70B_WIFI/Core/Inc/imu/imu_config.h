#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include "imu_types.h"

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

#define PALM_NODE_ID 0x30U
#define PALM_PROTOCOL_HEADER 0xB6U
#define PALM_PROTOCOL_FRAME_SIZE 35U
#define PALM_EXTERNAL_NODE_RX_ENABLE 1U
#define PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH 4U
#define PALM_EXTERNAL_NODE_MAX_LOCAL_ID 9U
#define PALM_EXTERNAL_NODE_UART2_BASE_ID 20U
#define PALM_EXTERNAL_NODE_UART3_BASE_ID 30U
#define PALM_EXTERNAL_NODE_UART4_BASE_ID 40U
#define PALM_EXTERNAL_NODE_UART5_BASE_ID 50U
#define PALM_EXTERNAL_NODE_UART6_BASE_ID 60U
#define PALM_EXTERNAL_NODE_UART7_BASE_ID 70U
#define PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY 0U
#define PALM_EXTERNAL_NODE_REMAP_RIGHT_MULTIPLY 1U

/*
 * Palm-side fallback remap for forwarded external fused quaternions.
 * Keep these at identity once fingertip firmware publishes the canonical frame.
 *
 * q_out = q_mount * q_in   when ORDER is LEFT_MULTIPLY
 * q_out = q_in * q_mount   when ORDER is RIGHT_MULTIPLY
 */

#define PALM_EXTERNAL_NODE_UART2_REMAP_W 1.0f
#define PALM_EXTERNAL_NODE_UART2_REMAP_X 0.0f
#define PALM_EXTERNAL_NODE_UART2_REMAP_Y 0.0f
#define PALM_EXTERNAL_NODE_UART2_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART2_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

#define PALM_EXTERNAL_NODE_UART3_REMAP_W 0.0f
#define PALM_EXTERNAL_NODE_UART3_REMAP_X 0.0f
#define PALM_EXTERNAL_NODE_UART3_REMAP_Y 1.0f
#define PALM_EXTERNAL_NODE_UART3_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART3_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

#define PALM_EXTERNAL_NODE_UART4_REMAP_W 0.0f
#define PALM_EXTERNAL_NODE_UART4_REMAP_X 1.0f
#define PALM_EXTERNAL_NODE_UART4_REMAP_Y 0.0f
#define PALM_EXTERNAL_NODE_UART4_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART4_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

#define PALM_EXTERNAL_NODE_UART5_REMAP_W 0.0f
#define PALM_EXTERNAL_NODE_UART5_REMAP_X 0.0f
#define PALM_EXTERNAL_NODE_UART5_REMAP_Y 1.0f
#define PALM_EXTERNAL_NODE_UART5_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART5_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

#define PALM_EXTERNAL_NODE_UART6_REMAP_W 1.0f
#define PALM_EXTERNAL_NODE_UART6_REMAP_X 0.0f
#define PALM_EXTERNAL_NODE_UART6_REMAP_Y 0.0f
#define PALM_EXTERNAL_NODE_UART6_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART6_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

#define PALM_EXTERNAL_NODE_UART7_REMAP_W 1.0f
#define PALM_EXTERNAL_NODE_UART7_REMAP_X 0.0f
#define PALM_EXTERNAL_NODE_UART7_REMAP_Y 0.0f
#define PALM_EXTERNAL_NODE_UART7_REMAP_Z 0.0f
#define PALM_EXTERNAL_NODE_UART7_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY

/*
 * Shared compile-time fingertip frame mapping applied to every forwarded
 * fingertip quaternion AFTER the per-UART remap above. This absorbs a
 * systematic rotation between the fingertip IMU's output frame and the
 * palm canonical glove frame. Keep this at identity once fingertip
 * firmware publishes the canonical glove frame directly.
 *
 * Semantics (sandwich conjugation, the correct operation for frame
 * correction, preserves rotation angle and rotates only the rotation axis):
 *   q_forwarded = q_common * (q_uart * q_raw) * conj(q_common)
 *
 * Default: 180 deg about Y composed with -90 deg about Z, i.e.
 *   q_common = q_180Y * q_-90Z = (0, -sqrt(1/2), +sqrt(1/2), 0)
 * which undoes a +90 deg yaw AND a 180 deg flip about the palm Y axis
 * between the fingertip board and the palm board (absorbs the observed
 * "X/Y swap plus X/Z inversion" pattern in one sandwich).
 *
 * Other useful presets:
 *   identity             : W = 1.0f,         X = 0, Y = 0, Z = 0
 *   -90 deg Z only       : W = 0.70710678f,  X = 0, Y = 0, Z = -0.70710678f
 *   +90 deg Z only       : W = 0.70710678f,  X = 0, Y = 0, Z = +0.70710678f
 *   180 deg Y only       : W = 0,            X = 0, Y = 1, Z = 0
 *   180 deg Y * +90 deg Z: W = 0,            X = +0.70710678f, Y = +0.70710678f, Z = 0
 */
#define PALM_EXTERNAL_NODE_COMMON_MAP_ENABLE 1U
#define PALM_EXTERNAL_NODE_COMMON_MAP_W  0.0f
#define PALM_EXTERNAL_NODE_COMMON_MAP_X -0.70710678f
#define PALM_EXTERNAL_NODE_COMMON_MAP_Y  0.70710678f
#define PALM_EXTERNAL_NODE_COMMON_MAP_Z  0.0f

/* Automatic runtime "Zero All" on boot. When enabled, the palm and every
 * forwarded fingertip port latch their first valid output as the zero
 * reference, so on power-up every quaternion stream starts near identity
 * without needing the USB Zero All command. */
#define PALM_GLOVE_ZERO_AT_STARTUP 1U

/* Delay from boot to the startup Zero All firing, in milliseconds.
 * Gives the palm Mahony filter time to finish gyro-bias calibration
 * (~4 s at 100 Hz) and the fingertip boards time to stream stable
 * quaternions before their references are latched. The capture itself is
 * still lazy per-source, so the palm only zeros once its first valid
 * fused attitude arrives and each fingertip port only zeros on its first
 * forwarded frame after this deadline. */
#define PALM_GLOVE_ZERO_STARTUP_DELAY_MS 5000U

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
#define PALM_MAHONY_KP 0.45f
#define PALM_MAHONY_KI 0.27f

#define PALM_GYRO_BIAS_CALIBRATION_SAMPLES 300U
#define PALM_GYRO_BIAS_CALIBRATION_TIMEOUT_SAMPLES 400U
#define PALM_GYRO_BIAS_MAX_STILL_DPS 1.5f
#define PALM_ACCEL_NORM_STILL_TOL_G 0.10f

/*
 * Canonical glove frame for every node:
 *   +X = forward toward the fingertips
 *   +Y = left across the hand
 *   +Z = up away from gravity when the hand is flat
 *
 * Palm IMU0 is mounted in the canonical frame.
 * Palm IMU1 is rotated 180 degrees around the board Z axis.
 */
#define PALM_IMU0_AXIS_REMAP_INIT { \
  IMU_AXIS_SOURCE_X,                \
  IMU_AXIS_SOURCE_Y,                \
  IMU_AXIS_SOURCE_Z,                \
  0U,                               \
  0U,                               \
  0U                                \
}

#define PALM_IMU1_AXIS_REMAP_INIT { \
  IMU_AXIS_SOURCE_X,                \
  IMU_AXIS_SOURCE_Y,                \
  IMU_AXIS_SOURCE_Z,                \
  1U,                               \
  1U,                               \
  0U                                \
}

/*
 * Palm output-frame remap applied to fused quaternion before USB serialization.
 * This matches the host-validated palm preview convention and allows host remap
 * to run as identity after firmware update.
 *
 * Axis map:
 *   X_out = -Y_in
 *   Y_out = +Z_in
 *   Z_out = -X_in
 *
 * Matrix form:
 *   v_out = C * v_in
 *   C = [ [ 0, -1,  0 ],
 *         [ 0,  0,  1 ],
 *         [ -1, 0,  0 ] ]
 *
 * Quaternion form (for R' = C * R * C^T):
 *   q_out = qC * q_in * conj(qC)
 */
#define PALM_OUTPUT_FRAME_REMAP_ENABLE 1U
#define PALM_OUTPUT_FRAME_REMAP_W  0.5f
#define PALM_OUTPUT_FRAME_REMAP_X -0.5f
#define PALM_OUTPUT_FRAME_REMAP_Y  0.5f
#define PALM_OUTPUT_FRAME_REMAP_Z  0.5f

#define PALM_IMU0_ACC_ORIENTATION "seu"
#define PALM_IMU0_GYRO_ORIENTATION "seu"
#define PALM_IMU1_ACC_ORIENTATION "seu"
#define PALM_IMU1_GYRO_ORIENTATION "seu"

#define PALM_IMU0_ALIGN_W 1.0f
#define PALM_IMU0_ALIGN_X 0.0f
#define PALM_IMU0_ALIGN_Y 0.0f
#define PALM_IMU0_ALIGN_Z 0.0f

#define PALM_IMU1_ALIGN_W 1.0f
#define PALM_IMU1_ALIGN_X 0.0f
#define PALM_IMU1_ALIGN_Y 0.0f
#define PALM_IMU1_ALIGN_Z 0.0f

/* Legacy compatibility for files still present in the project tree. */
#define PALM_FUSION_MAX_DISAGREEMENT_DEG 20.0f

#define PALM_MOTIONFX_GBIAS_ACC_TH_SC 0.00153f
#define PALM_MOTIONFX_GBIAS_GYRO_TH_SC 0.004f
#define PALM_MOTIONFX_GBIAS_MAG_TH_SC 1.0f

#define PALM_STATUS_IMU0_OK           (1U << 0)
#define PALM_STATUS_IMU1_OK           (1U << 1)
#define PALM_STATUS_FUSION_READY      (1U << 2)
#define PALM_STATUS_SINGLE_IMU_MODE   (1U << 3)
#define PALM_STATUS_USB_BUSY          (1U << 4)
#define PALM_STATUS_CALIBRATING       (1U << 5)
#define PALM_STATUS_FILTER_WARMUP     (1U << 6)

#endif /* IMU_CONFIG_H */
