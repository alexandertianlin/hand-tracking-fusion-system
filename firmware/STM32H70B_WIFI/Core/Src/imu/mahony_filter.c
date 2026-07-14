#include "imu/mahony_filter.h"

#include <math.h>
#include <string.h>
#include "imu/imu_config.h"


#define MAHONY_RAD_PER_DEG (3.14159265f / 180.0f)


uint8_t  dbg_gravity_trusted = 1;

float dbg_comp_x = 0.0f;
float dbg_comp_y = 0.0f;
float dbg_comp_z = 0.0f;

static float cached_err_x = 0.0f;
static float cached_err_y = 0.0f;
static float cached_err_z = 0.0f;

static float mahony_filter_inv_sqrt(float value)
{
  if (value <= 0.0f) {
    return 0.0f;
  }
  return 1.0f / sqrtf(value);
}

static void mahony_filter_normalize_quaternion(imu_quatf_t *quaternion)
{
  float inv_norm = mahony_filter_inv_sqrt((quaternion->w * quaternion->w) +
                                          (quaternion->x * quaternion->x) +
                                          (quaternion->y * quaternion->y) +
                                          (quaternion->z * quaternion->z));

  if (inv_norm <= 0.0f) {
    quaternion->w = 1.0f;
    quaternion->x = 0.0f;
    quaternion->y = 0.0f;
    quaternion->z = 0.0f;
    return;
  }

  quaternion->w *= inv_norm;
  quaternion->x *= inv_norm;
  quaternion->y *= inv_norm;
  quaternion->z *= inv_norm;

  if (quaternion->w < 0.0f) {
    quaternion->w = -quaternion->w;
    quaternion->x = -quaternion->x;
    quaternion->y = -quaternion->y;
    quaternion->z = -quaternion->z;
  }
}

static float mahony_filter_clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static void mahony_filter_update_euler(const imu_quatf_t *quaternion,
                                       imu_vec3f_t *euler_deg)
{
  float sinr_cosp;
  float cosr_cosp;
  float sinp;
  float siny_cosp;
  float cosy_cosp;

  sinr_cosp = 2.0f * ((quaternion->w * quaternion->x) + (quaternion->y * quaternion->z));
  cosr_cosp = 1.0f - (2.0f * ((quaternion->x * quaternion->x) + (quaternion->y * quaternion->y)));
  euler_deg->x = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265f);

  sinp = 2.0f * ((quaternion->w * quaternion->y) - (quaternion->z * quaternion->x));
  sinp = mahony_filter_clamp(sinp, -1.0f, 1.0f);
  euler_deg->y = asinf(sinp) * (180.0f / 3.14159265f);

  siny_cosp = 2.0f * ((quaternion->w * quaternion->z) + (quaternion->x * quaternion->y));
  cosy_cosp = 1.0f - (2.0f * ((quaternion->y * quaternion->y) + (quaternion->z * quaternion->z)));
  euler_deg->z = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159265f);
}

HAL_StatusTypeDef mahony_filter_init(mahony_filter_t *filter)
{
  if (filter == NULL) {
    return HAL_ERROR;
  }

  memset(filter, 0, sizeof(*filter));
  filter->quaternion.w = 1.0f;

  filter->two_kp = 2.0f * PALM_MAHONY_KP;
  filter->two_ki = 2.0f * PALM_MAHONY_KI;

  filter->warmup_remaining = PALM_IMU_WARMUP_SAMPLES;
  filter->initialized = 1U;

  cached_err_x = 0.0f;
  cached_err_y = 0.0f;
  cached_err_z = 0.0f;

  return HAL_OK;
}

HAL_StatusTypeDef mahony_filter_update(mahony_filter_t *filter,
                                       const imu_sample_t *sample,
                                       float delta_time_s,
                                       imu_attitude_t *attitude)
{
  if ((filter == NULL) || (sample == NULL) || (attitude == NULL) || (filter->initialized == 0U)) {
    return HAL_ERROR;
  }

  float ax = sample->accel_g.x;
  float ay = sample->accel_g.y;
  float az = sample->accel_g.z;
  float gx, gy, gz;
  float vx, vy, vz;
  float error_x = 0.0f, error_y = 0.0f, error_z = 0.0f;
  float qw, qx, qy, qz;

  const float GYRO_STILL_THRESH = 0.5f;
  const float ACC_NORM_LOW  = 0.75f;
  const float ACC_NORM_HIGH = 1.25f;

  const float EMA_ALPHA = 0.005f;

  if (delta_time_s <= 0.0f) {
    delta_time_s = PALM_IMU_FILTER_DT_S;
  }

  qw = filter->quaternion.w;
  qx = filter->quaternion.x;
  qy = filter->quaternion.y;
  qz = filter->quaternion.z;

  vx = 2.0f * (qx * qz - qw * qy);
  vy = 2.0f * (qw * qx + qy * qz);
  vz = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  // 1. 隔离真实的传感器读数 (注意：这里已经是外层扣除过开机零偏的数据了)
  float raw_gx_rad = sample->gyro_dps.x * MAHONY_RAD_PER_DEG;
  float raw_gy_rad = sample->gyro_dps.y * MAHONY_RAD_PER_DEG;
  float raw_gz_rad = sample->gyro_dps.z * MAHONY_RAD_PER_DEG;

  float gyro_norm = sqrtf(raw_gx_rad*raw_gx_rad + raw_gy_rad*raw_gy_rad + raw_gz_rad*raw_gz_rad);
  float accel_norm = sqrtf(ax*ax + ay*ay + az*az);

  uint8_t gyro_still  = (gyro_norm < GYRO_STILL_THRESH) ? 1 : 0;
  uint8_t accel_valid = (accel_norm >= ACC_NORM_LOW && accel_norm <= ACC_NORM_HIGH) ? 1 : 0;
  uint8_t gravity_ok  = (gyro_still && accel_valid) ? 1 : 0;

  // 2. 【核心隔离逻辑】静止时更新动态零偏 (ZUPT)

  if (gravity_ok) {
    // 利用真实的 raw 数据持续更新动态零偏，使用 EMA 替代大数组平均
    filter->dynamic_bias_x = (1.0f - EMA_ALPHA) * filter->dynamic_bias_x + EMA_ALPHA * raw_gx_rad;
    filter->dynamic_bias_y = (1.0f - EMA_ALPHA) * filter->dynamic_bias_y + EMA_ALPHA * raw_gy_rad;
    filter->dynamic_bias_z = (1.0f - EMA_ALPHA) * filter->dynamic_bias_z + EMA_ALPHA * raw_gz_rad;
  }

  if (accel_norm > 0.1f) {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;
  }

  float current_two_kp = filter->two_kp;

  if (!gyro_still)//如果运动
  {
    // 3A. 运动状态：扣除 EMA 学习到的动态零偏，赋给参与运算的 gx/gy/gz
    dbg_gravity_trusted = 0;
    error_x = 0.0f; error_y = 0.0f; error_z = 0.0f;

    gx = raw_gx_rad; //- filter->dynamic_bias_x;
    gy = raw_gy_rad; //- filter->dynamic_bias_y;
    gz = raw_gz_rad; //- filter->dynamic_bias_z;

    filter->recovery_counter = 240;
  }
  else
  {

    dbg_gravity_trusted = gravity_ok;
    gx = 0.0f;
    gy = 0.0f;
    gz = 0.0f;

    // 依然保留加速度计的纠偏能力，把累积的极小误差拉回水平
    if(accel_valid)//加速度可信
    {
      error_x = (ay * vz) - (az * vy);
      error_y = (az * vx) - (ax * vz);
      error_z = (ax * vy) - (ay * vx);

      if (filter->recovery_counter > 0) {
          filter->recovery_counter--;
          current_two_kp = filter->two_kp * 5.0f;
      }
    }
    else//不可信就不修正
    {
      error_x = 0.0f;
      error_y = 0.0f;
      error_z = 0.0f;
    }
  }

  // 4. Mahony 内部积分：静止时不积分，避免与 EMA 零偏学习冲突产生 Windup
  if (filter->two_ki > 0.0f && !gyro_still) {//ki
    filter->integral_error.x += filter->two_ki * error_x * delta_time_s;
    filter->integral_error.y += filter->two_ki * error_y * delta_time_s;
    filter->integral_error.z += filter->two_ki * error_z * delta_time_s;
  }

  // 5. 融合修正  //kp
  gx += current_two_kp * error_x + filter->integral_error.x;
  gy += current_two_kp * error_y + filter->integral_error.y;
  gz += current_two_kp * error_z + filter->integral_error.z;

  // 6. 四元数与欧拉角更新 (代码保持不变)
  float half_dt = 0.5f * delta_time_s;
  filter->quaternion.w += (-qx*gx - qy*gy - qz*gz) * half_dt;
  filter->quaternion.x += (qw*gx + qy*gz - qz*gy) * half_dt;
  filter->quaternion.y += (qw*gy - qx*gz + qz*gx) * half_dt;
  filter->quaternion.z += (qw*gz + qx*gy - qy*gx) * half_dt;

  mahony_filter_normalize_quaternion(&filter->quaternion);
  attitude->quaternion = filter->quaternion;
  mahony_filter_update_euler(&attitude->quaternion, &attitude->euler_deg);

  const float rad2deg = 180.0f / 3.14159265f;
  dbg_comp_x = filter->integral_error.x * rad2deg;
  dbg_comp_y = filter->integral_error.y * rad2deg;
  dbg_comp_z = filter->integral_error.z * rad2deg;

  if (filter->warmup_remaining > 0U) {
    filter->warmup_remaining--;
    attitude->valid = 0U;
    return HAL_BUSY;
  }

  attitude->valid = 1U;
  return HAL_OK;
}
