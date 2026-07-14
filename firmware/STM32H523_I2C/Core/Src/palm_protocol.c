#include "protocol/palm_protocol.h"

#include <limits.h>
#include <string.h>

static int16_t palm_protocol_clamp_to_i16(float value)
{
  if (value > (float)INT16_MAX) {
    return INT16_MAX;
  }

  if (value < (float)INT16_MIN) {
    return INT16_MIN;
  }

  return (int16_t)value;
}

static void palm_protocol_put_i16_le(uint8_t *dst, int16_t value)
{
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
}

static uint8_t palm_protocol_crc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  uint8_t index;

  for (index = 0U; index < len; index++) {
    crc ^= data[index];
  }

  return crc;
}
/*
void palm_protocol_build_frame(const palm_protocol_payload_t *payload,
                               uint8_t frame[PALM_PROTOCOL_FRAME_SIZE])
{
  frame[0] = PALM_PROTOCOL_HEADER;

  palm_protocol_put_i16_le(&frame[1], 0);
  palm_protocol_put_i16_le(&frame[3], 0);
  palm_protocol_put_i16_le(&frame[5], 0);

  palm_protocol_put_i16_le(&frame[7], palm_protocol_clamp_to_i16(payload->orientation.w * PALM_QUAT_SCALE));
  palm_protocol_put_i16_le(&frame[9], palm_protocol_clamp_to_i16(payload->orientation.x * PALM_QUAT_SCALE));
  palm_protocol_put_i16_le(&frame[11], palm_protocol_clamp_to_i16(payload->orientation.y * PALM_QUAT_SCALE));
  palm_protocol_put_i16_le(&frame[13], palm_protocol_clamp_to_i16(payload->orientation.z * PALM_QUAT_SCALE));

  frame[15] = payload->status;
  frame[16] = payload->node_id;
  frame[17] = palm_protocol_crc(&frame[1], 16U);
}*/
/*
void palm_protocol_build_frame(const palm_protocol_payload_t *payload,
                               uint8_t frame[PALM_PROTOCOL_FRAME_SIZE])
{
    // 帧头
    frame[0] = 0xB5;
    frame[1] = 0xA5;
    frame[2] = 0x55;

    uint16_t total_len = 0;
    palm_protocol_put_i16_le(&frame[3], 0);

    frame[5] = payload->model_id;
    frame[6] = payload->node_id;
    frame[7] = payload->status;

    uint8_t payload_ptr = 8;

    // 打包四元数（w, x, y, z）
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.w * PALM_QUAT_SCALE));
    payload_ptr += 2;

    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.x * PALM_QUAT_SCALE));
    payload_ptr += 2;

    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.y * PALM_QUAT_SCALE));
    payload_ptr += 2;

    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.z * PALM_QUAT_SCALE));
    payload_ptr += 2;

    // 打包加速度（x, y, z）
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.x * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;

    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.y * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;

    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.z * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;

    total_len = payload_ptr + 1;
    palm_protocol_put_i16_le(&frame[3], total_len);

    uint8_t crc = 0;
    for (uint8_t i = 0; i < payload_ptr; i++) {
        crc ^= frame[i];
    }
    frame[payload_ptr] = crc;
}*/

void palm_protocol_build_frame(const palm_protocol_payload_t *payload,
                               uint8_t frame[PALM_PROTOCOL_FRAME_SIZE])
{
    // 帧头 不变
    frame[0] = 0xB5;
    frame[1] = 0xA5;
    frame[2] = 0x55;

    uint16_t total_len = 0;
    palm_protocol_put_i16_le(&frame[3], 0);

    frame[5] = payload->model_id;
    frame[6] = payload->node_id;
    frame[7] = payload->status;

    uint8_t payload_ptr = 8;

    // ====================== 原有IMU四元数 ======================
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.w * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.x * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.y * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->orientation.z * PALM_QUAT_SCALE));
    payload_ptr += 2;

    // ====================== 原有IMU加速度 ======================
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.x * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.y * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(payload->accel_g.z * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;

    memcpy(&frame[payload_ptr], &payload->force_x, 4);
    payload_ptr += 4;
    memcpy(&frame[payload_ptr], &payload->force_y, 4);
    payload_ptr += 4;
    memcpy(&frame[payload_ptr], &payload->force_z, 4);
    payload_ptr += 4;

    total_len = payload_ptr + 1;
    palm_protocol_put_i16_le(&frame[3], total_len);

    uint8_t crc = 0;
    for (uint8_t i = 0; i < payload_ptr; i++) {
        crc ^= frame[i];
    }
    frame[payload_ptr] = crc;
}

void palm_protocol_build_raw_frame(const palm_protocol_raw_payload_t *payload,
                                   uint8_t frame[PALM_RAW_PROTOCOL_FRAME_SIZE])
{
  uint8_t offset = 3U;
  uint8_t imu_index;

  frame[0] = PALM_RAW_PROTOCOL_HEADER;
  frame[1] = PALM_RAW_PROTOCOL_TYPE;
  frame[2] = payload->sequence;

  for (imu_index = 0U; imu_index < PALM_IMU_COUNT; imu_index++) {
    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].accel_g.x * PALM_RAW_ACCEL_MG_SCALE));
    offset += 2U;
    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].accel_g.y * PALM_RAW_ACCEL_MG_SCALE));
    offset += 2U;
    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].accel_g.z * PALM_RAW_ACCEL_MG_SCALE));
    offset += 2U;

    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].gyro_dps.x * PALM_RAW_GYRO_DPS_X10_SCALE));
    offset += 2U;
    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].gyro_dps.y * PALM_RAW_GYRO_DPS_X10_SCALE));
    offset += 2U;
    palm_protocol_put_i16_le(&frame[offset], palm_protocol_clamp_to_i16(payload->imu_samples[imu_index].gyro_dps.z * PALM_RAW_GYRO_DPS_X10_SCALE));
    offset += 2U;
  }

  frame[offset++] = payload->status;
  frame[offset] = palm_protocol_crc(&frame[1], PALM_RAW_PROTOCOL_FRAME_SIZE - 2U);
}
