#ifndef MOTIONFX_WRAPPER_H
#define MOTIONFX_WRAPPER_H

#include "main.h"
#include "imu/imu_types.h"
#include <stddef.h>
#include "motion_fx.h"

#define MOTIONFX_WRAPPER_STATE_BYTES 4096U

#define MOTIONFX_DIAG_STAGE_IDLE          0U
#define MOTIONFX_DIAG_STAGE_ENABLE_CRC    1U
#define MOTIONFX_DIAG_STAGE_GET_SIZE      2U
#define MOTIONFX_DIAG_STAGE_INITIALIZE    3U
#define MOTIONFX_DIAG_STAGE_GET_KNOBS     4U
#define MOTIONFX_DIAG_STAGE_SET_KNOBS     5U
#define MOTIONFX_DIAG_STAGE_ENABLE_6X     6U
#define MOTIONFX_DIAG_STAGE_ENABLE_9X     7U
#define MOTIONFX_DIAG_STAGE_READY         8U
#define MOTIONFX_DIAG_STAGE_FAILED        9U

typedef struct {
  uint32_t state_words[MOTIONFX_WRAPPER_STATE_BYTES / sizeof(uint32_t)] __attribute__((aligned(16)));
  MFX_knobs_t knobs;
  uint16_t required_state_bytes;
  uint16_t warmup_remaining;
  uint8_t initialized;
} motionfx_wrapper_t;

extern volatile uint32_t g_motionfx_diag_stage;
extern volatile uint32_t g_motionfx_diag_last_status;
extern volatile uint32_t g_motionfx_diag_state_size;

HAL_StatusTypeDef motionfx_wrapper_init(motionfx_wrapper_t *wrapper,
                                        const char *acc_orientation,
                                        const char *gyro_orientation);
HAL_StatusTypeDef motionfx_wrapper_update(motionfx_wrapper_t *wrapper,
                                          const imu_sample_t *sample,
                                          float delta_time_s,
                                          imu_attitude_t *attitude);

#endif /* MOTIONFX_WRAPPER_H */
