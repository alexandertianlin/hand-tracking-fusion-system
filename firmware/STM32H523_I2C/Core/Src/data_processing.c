#include "data_processing.h"
#include "string.h"
#include "stm32h5xx_it.h"
#include "float.h"
#include "math.h"
#include <ctype.h>

uint8_t working_mode = 1;
extern const char model_param[16];
extern UART_HandleTypeDef huart1;

// 合并所有状态变量（每个传感器通道独立）
sensor_processing_state_t sensor_processing_state[ARRAY_PS];

// 处理后的数据（全局，实际使用时直接读取 data_temp）
SensorData sensor_processed_data = {0};
SensorData fitting_sensor_processed_data = {0};

// ==================== 型号适配阈值 ====================
typedef struct {
    float xy_small;
    float xy_medium;
    float xy_large;
    float z_small;
    float z_medium;
    float z_large;
} SensorThresholds;

typedef struct {
    const char* sensor_type;
    SensorThresholds thresholds;
} SensorConfig;

SensorConfig sensor_configs[] = {
    {"SSO-N100T10", {20.0f, 40.0f, 150.0f, 1.5f, 5.0f, 15.0f}},
    {"SMR-N100T10", {40.0f, 100.0f, 150.0f, 1.5f, 5.0f, 7.5f}},
    {"SP-N20T10",   {20.0f, 40.0f, 150.0f, 1.5f, 5.0f, 15.0f}},
    {"SF-N20T10",   {40.0f, 100.0f, 150.0f, 1.5f, 5.0f, 7.5f}},
};

SensorThresholds default_thresholds = {20.0f, 40.0f, 150.0f, 1.5f, 5.0f, 15.0f};

// 获取传感器阈值
SensorThresholds get_sensor_thresholds(const char* sensor_type) {
    int num = sizeof(sensor_configs) / sizeof(sensor_configs[0]);
    for (int i = 0; i < num; i++) {
        if (strcmp(sensor_type, sensor_configs[i].sensor_type) == 0)
            return sensor_configs[i].thresholds;
    }
    return default_thresholds;
}

// ==================== 系统配置参数 ====================
typedef struct {
    float X_T, Y_T, Z_T;
    int32_t X_f, Y_f;
    int Z_f;
    int ActuallongWindowSize;
    int ActualDATA_CheckSIZE;
    int ActualCheckWINDOW_SIZE;
} system_config_t;

static const system_config_t config = {
    .X_T = 1.202f, .Y_T = 1.202f, .Z_T = 1.936f,
    .X_f = 200, .Y_f = 200, .Z_f = 10,
    .ActuallongWindowSize = 100,
    .ActualDATA_CheckSIZE = 20,
    .ActualCheckWINDOW_SIZE = 10
};

// 常量定义
#define STILL_DEBOUNCE_COUNT(size)    ((int)(1.5 * (size)))
#define STILL_RELEASE_COUNT(size)     ((int)(0.05 * (size)))
#define STABLE_THRESHOLD_Z            5.0f
#define STABLE_THRESHOLD_XY           8.0f
#define MIN_FORCE_Z                   0.5f

// ==================== 工具函数 ====================
static inline float update_min(float min_val, float new_val) {
    return (new_val < min_val) ? new_val : min_val;
}
static inline float update_max(float max_val, float new_val) {
    return (new_val > max_val) ? new_val : max_val;
}

// ==================== 核心处理函数（多通道版本） ====================

// 长窗口滑动平均及偏差计算
void update_sliding_window(sensor_processing_state_t* state, float x, float y, float z) {
    if (state->FluentWindowfirstflag == 0) {
        for (int s = 0; s < config.ActuallongWindowSize; s++) {
            state->X_longwindow[s] = x;
            state->Y_longwindow[s] = y;
            state->Z_longwindow[s] = z;
        }
        state->X_long_sum = x * config.ActuallongWindowSize;
        state->Y_long_sum = y * config.ActuallongWindowSize;
        state->Z_long_sum = z * config.ActuallongWindowSize;
        state->FluentWindowfirstflag = 1;
    } else {
        uint16_t idx = state->long_idx;
        state->X_long_sum -= state->X_longwindow[idx];
        state->Y_long_sum -= state->Y_longwindow[idx];
        state->Z_long_sum -= state->Z_longwindow[idx];

        state->X_longwindow[idx] = x;
        state->Y_longwindow[idx] = y;
        state->Z_longwindow[idx] = z;

        state->X_long_sum += x;
        state->Y_long_sum += y;
        state->Z_long_sum += z;
    }
    state->long_idx = (state->long_idx + 1) % config.ActuallongWindowSize;

    state->X_long_mean = state->X_long_sum / config.ActuallongWindowSize;
    state->Y_long_mean = state->Y_long_sum / config.ActuallongWindowSize;
    state->Z_long_mean = state->Z_long_sum / config.ActuallongWindowSize;

    state->x_dev2 = x - state->X_long_mean;
    state->y_dev2 = y - state->Y_long_mean;
    state->z_dev2 = z - state->Z_long_mean;
}

// 环形缓冲滑动平均（平滑滤波）
void process_sensor_data_point(sensor_processing_state_t* state, float* x, float* y, float* z) {
    state->xtotal -= state->xsmoothwindow[state->smoothIndex];
    state->xsmoothwindow[state->smoothIndex] = *x;
    state->xtotal += *x;

    state->ytotal -= state->ysmoothwindow[state->smoothIndex];
    state->ysmoothwindow[state->smoothIndex] = *y;
    state->ytotal += *y;

    state->ztotal -= state->zsmoothwindow[state->smoothIndex];
    state->zsmoothwindow[state->smoothIndex] = *z;
    state->ztotal += *z;

    state->smoothIndex = (state->smoothIndex + 1) % OPTIMIZED_SMOOTH_WINDOW_SIZE;

    if (state->size < OPTIMIZED_SMOOTH_WINDOW_SIZE) {
        state->size++;
        *x = state->xtotal / state->size;
        *y = state->ytotal / state->size;
        *z = state->ztotal / state->size;
    } else {
        *x = state->xtotal / OPTIMIZED_SMOOTH_WINDOW_SIZE;
        *y = state->ytotal / OPTIMIZED_SMOOTH_WINDOW_SIZE;
        *z = state->ztotal / OPTIMIZED_SMOOTH_WINDOW_SIZE;
    }
}

// 传感器非线性校准（与 single 相同）
void apply_sensor_calibration_point(float* x, float* y, float* z, uint8_t sensor_idx) {


//    extern double	a_x_0, b_x_0, c_x_0, a_y_0, b_y_0, c_y_0, a_z_0, b_z_0, c_z_0;
//    extern double 	K0_0, K1_0, K2_0, K3_0, K4_0, K5_0, K6_0, K7_0, K8_0, K9_0;

	double K0_0 = 1;
	double K1_0 = 1;
	double K2_0 = 1;
	double K3_0 = 1;
	double K4_0 = 1;
	double K5_0 = 1;
	double K6_0 = 1;
	double K7_0 = 1;
	double K8_0 = 1;
	double K9_0 = 1;

	double a_x_0 = 0;
	double b_x_0 = 0;
	double c_x_0 = 1;

	double a_y_0 = 0;
	double b_y_0 = 0;
	double c_y_0 = 1;

	double a_z_0 = 0;
	double b_z_0 = 0;
	double c_z_0 = 1;

//    extern double 	a_x_1, b_x_1, c_x_1, a_y_1, b_y_1, c_y_1, a_z_1, b_z_1, c_z_1;
//    extern double 	K0_1, K1_1, K2_1, K3_1, K4_1, K5_1, K6_1, K7_1, K8_1, K9_1;

	double a_x, b_x, c_x, a_y, b_y, c_y, a_z, b_z, c_z;
	double K0, K1, K2, K3, K4, K5, K6, K7, K8, K9;

	switch (sensor_idx) {
	        case 0:
	            a_x = a_x_0; b_x = b_x_0; c_x = c_x_0;
	            a_y = a_y_0; b_y = b_y_0; c_y = c_y_0;
	            a_z = a_z_0; b_z = b_z_0; c_z = c_z_0;
	            K0 = K0_0; K1 = K1_0; K2 = K2_0; K3 = K3_0; K4 = K4_0;
	            K5 = K5_0; K6 = K6_0; K7 = K7_0; K8 = K8_0; K9 = K9_0;
	            break;
//	        case 1:
//	            a_x = a_x_1; b_x = b_x_1; c_x = c_x_1;
//	            a_y = a_y_1; b_y = b_y_1; c_y = c_y_1;
//	            a_z = a_z_1; b_z = b_z_1; c_z = c_z_1;
//	            K0 = K0_1; K1 = K1_1; K2 = K2_1; K3 = K3_1; K4 = K4_1;
//	            K5 = K5_1; K6 = K6_1; K7 = K7_1; K8 = K8_1; K9 = K9_1;
//	            break;
	        default:
	            return;  // 无效索引，直接退出
	    }

    // 1. 额外的 Z 串扰补偿
//    if ((fabsf(*x) >= 0.2f) || (fabsf(*y) >= 0.2f)) {
//        *z = (float)(K0 + K1 * (*x) + K2 * (*y) + K3 * (*z) + K4 * (*y) * (*y)
//                   + K5 * (*x) * (*y) + K6 * (*x) * (*z) + K7 * (*y) * (*y)
//                   + K8 * (*y) * (*z) + K9 * (*z) * (*z));
//    }

    // 2. 负 Z 处理
    if ((*z < 0) && (working_mode == 1 || working_mode == 3)) {
        *z = 0.0f;
    }

    // 3. 非线性校准
    float tmp_x = *x;
    *x = (float)(a_x * tmp_x * tmp_x * tmp_x + b_x * tmp_x * tmp_x + c_x * tmp_x);
    float tmp_y = *y;
    *y = (float)(a_y * tmp_y * tmp_y * tmp_y + b_y * tmp_y * tmp_y + c_y * tmp_y);
    float tmp_z = *z;
    *z = (float)(a_z * tmp_z * tmp_z * tmp_z + b_z * tmp_z * tmp_z + c_z * tmp_z);

    // 4. 有限性检查
    if (!isfinite(*x)) *x = 0.0f;
    if (!isfinite(*y)) *y = 0.0f;
    if (!isfinite(*z)) *z = 0.0f;
}


// 数据校验（用于清零条件）
static void process_data_check(sensor_processing_state_t* state,
                               float x_threshold, float y_threshold, float z_threshold,
                               uint8_t check_z) {
    if (state->Current_CheckNum < config.ActualDATA_CheckSIZE) {
        state->XDATA_Check[state->Current_CheckNum] = state->current_x;
        state->YDATA_Check[state->Current_CheckNum] = state->current_y;
        if (check_z)
            state->ZDATA_Check[state->Current_CheckNum] = state->current_z;
        state->Current_CheckNum++;
    }

    if (state->Current_CheckNum == config.ActualDATA_CheckSIZE) {
        int start = config.ActualDATA_CheckSIZE - config.ActualCheckWINDOW_SIZE;
        float x_max = state->XDATA_Check[start];
        float x_min = x_max;
        float y_max = state->YDATA_Check[start];
        float y_min = y_max;
        float z_max = 0, z_min = 0;
        if (check_z) {
            z_max = state->ZDATA_Check[start];
            z_min = z_max;
        }

        for (int s = start; s < config.ActualDATA_CheckSIZE; s++) {
            x_max = update_max(x_max, state->XDATA_Check[s]);
            x_min = update_min(x_min, state->XDATA_Check[s]);
            y_max = update_max(y_max, state->YDATA_Check[s]);
            y_min = update_min(y_min, state->YDATA_Check[s]);
            if (check_z) {
                z_max = update_max(z_max, state->ZDATA_Check[s]);
                z_min = update_min(z_min, state->ZDATA_Check[s]);
            }
        }

        if (check_z) {
            if ((x_max < x_threshold) && (x_min > -x_threshold) &&
                (y_max < y_threshold) && (y_min > -y_threshold) &&
                (z_min > -z_threshold) && (z_max < z_threshold)) {
                state->only_zclear = 1;
                clear_function_point(state);
            } else {
                state->Current_CheckNum = 0;
            }
        } else {
            if ((x_max < x_threshold) && (x_min > -x_threshold) &&
                (y_max < y_threshold) && (y_min > -y_threshold)) {
                state->only_zclear = 1;
                clear_function_point(state);
            } else {
                state->Current_CheckNum = 0;
            }
        }
    }
}

// 处理 Z 轴各种状态（标准/抗磁模式使用）
static void handle_z_up_states(sensor_processing_state_t* state, const char* sensor_type) {
    SensorThresholds thresholds = get_sensor_thresholds(sensor_type);
    float xy_small = thresholds.xy_small;
    float xy_medium = thresholds.xy_medium;
    float xy_large = thresholds.xy_large;
    float z_small = thresholds.z_small;
    float z_medium = thresholds.z_medium;
    float z_large = thresholds.z_large;

    switch (state->z_up_flag) {
        case -2:
            state->current_z = 0;
            if (fabsf(state->x_dev2) < 8 && fabsf(state->y_dev2) < 8) {
                if (fabsf(state->current_x) <= xy_small) state->current_x = 0;
                if (fabsf(state->current_y) <= xy_small) state->current_y = 0;
                process_data_check(state, 20.0f, 20.0f, 0, 0);
            }
            break;
        case -3:
            state->current_z = 0;
            if (fabsf(state->x_dev2) < 8 && fabsf(state->y_dev2) < 8) {
                if (fabsf(state->current_x) <= xy_medium) state->current_x = 0;
                if (fabsf(state->current_y) <= xy_medium) state->current_y = 0;
                process_data_check(state, 20.0f, 20.0f, 0, 0);
            }
            break;
        case -4:
            state->current_z = 0;
            if (fabsf(state->x_dev2) < 50 && fabsf(state->y_dev2) < 50) {
                if (fabsf(state->current_x) <= xy_large) state->current_x = 0;
                if (fabsf(state->current_y) <= xy_large) state->current_y = 0;
                process_data_check(state, 100.0f, 100.0f, 0, 0);
            }
            break;
        case 1:
            if (fabsf(state->z_dev2) > 0) {
                if (fabsf(state->current_x) <= 20 && fabsf(state->current_y) <= 20 && fabsf(state->current_z) <= 5)
                    state->current_x = state->current_y = state->current_z = 0;
                else {
                    if (fabsf(state->current_x) <= 21) state->current_x = 0;
                    if (fabsf(state->current_y) <= 21) state->current_y = 0;
                    if (fabsf(state->current_z) <= z_small) state->current_z = 0;
                }
                process_data_check(state, 10.0f, 10.0f, 1.0f, 1);
            }
            break;
        case 2:
            if (fabsf(state->z_dev2) > 0) {
                if (fabsf(state->current_x) <= 35 && fabsf(state->current_y) <= 35 && fabsf(state->current_z) <= 10)
                    state->current_x = state->current_y = state->current_z = 0;
                else {
                    if (fabsf(state->current_x) <= 50) state->current_x = 0;
                    if (fabsf(state->current_y) <= 50) state->current_y = 0;
                    if (fabsf(state->current_z) <= z_medium) state->current_z = 0;
                }
                process_data_check(state, 30.0f, 30.0f, 3.0f, 1);
            }
            break;
        case 3:
            if (fabsf(state->z_dev2) > 0) {
                if ((fabsf(state->current_x) <= 45 && fabsf(state->current_y) <= 45 && fabsf(state->current_z) <= 15) ||
                    (fabsf(state->current_x) <= 75 && fabsf(state->current_y) <= 75 && fabsf(state->current_z) <= 10))
                    state->current_x = state->current_y = state->current_z = 0;
                else {
                    if (fabsf(state->current_x) <= 100) state->current_x = 0;
                    if (fabsf(state->current_y) <= 100) state->current_y = 0;
                    if (fabsf(state->current_z) <= z_large) state->current_z = 0;
                }
                process_data_check(state, 85.0f, 85.0f, 10.0f, 1);
            }
            break;
        default:
            if (state->z_up_flag == 0)
                state->current_x = state->current_y = state->current_z = 0;
            break;
    }
}

// 静止检测与保持
static void update_still_detection(sensor_processing_state_t* state) {
    if (fabsf(state->z_dev2) < STABLE_THRESHOLD_Z &&
        fabsf(state->x_dev2) < STABLE_THRESHOLD_XY &&
        fabsf(state->y_dev2) < STABLE_THRESHOLD_XY &&
        fabsf(state->current_z) > MIN_FORCE_Z) {

        state->still_flag++;
        state->still_release_count = 0;

        if (state->still_flag > STILL_DEBOUNCE_COUNT(config.ActuallongWindowSize)) {
            if (state->still_update_flag == 0) {
                state->still.x0 = state->current_x;
                state->still.y0 = state->current_y;
                state->still.z0 = state->current_z;
                state->still_update_flag = 1;
                state->stillgap.x0 = state->stillgap.y0 = state->stillgap.z0 = 0.0f;
            } else {
                state->current_x = state->still.x0;
                state->current_y = state->still.y0;
                state->current_z = state->still.z0;
                float adjust = 0.001f;
                state->still.x0 = state->still.x0 * (1 - adjust) + state->current_x * adjust;
                state->still.y0 = state->still.y0 * (1 - adjust) + state->current_y * adjust;
                state->still.z0 = state->still.z0 * (1 - adjust) + state->current_z * adjust;
            }
        }
    } else {
        if (state->still_update_flag == 1) {
            state->still_release_count++;
            if (state->still_release_count > STILL_RELEASE_COUNT(config.ActuallongWindowSize)) {
                state->stillgap.x0 = state->still.x0 - state->current_x;
                state->stillgap.y0 = state->still.y0 - state->current_y;
                state->stillgap.z0 = state->still.z0 - state->current_z;
                state->still_update_flag = 0;
                state->still_flag = 0;
            } else {
                state->current_x = state->still.x0;
                state->current_y = state->still.y0;
                state->current_z = state->still.z0;
            }
        } else {
            if (state->current_z > 0.1f) {
                state->current_x += state->stillgap.x0;
                state->current_y += state->stillgap.y0;
                state->current_z += state->stillgap.z0;
            }
        }
    }
}

// 清零条件检测
static void update_clear_conditions(sensor_processing_state_t* state, const char* sensor_type) {
    // Z轴清零条件
    if (state->current_z < 0) {
        if (fabsf(state->current_x) <= 0.08f && fabsf(state->current_y) <= 0.08f) {
            state->zclearflag1++;
            if (state->zclearflag1 > 10 * config.ActuallongWindowSize) {
                state->only_zclear = 1;
                clear_function_point(state);
            }
        } else {
            state->zclearflag1 = 0;
        }

        if (fabsf(state->current_x) <= 0.04f && fabsf(state->current_y) <= 0.04f) {
            state->zclearflag2++;
            if (state->zclearflag2 > 5.0f * config.ActuallongWindowSize) {
                state->only_zclear = 1;
                clear_function_point(state);
            }
        } else {
            state->zclearflag2 = 0;
        }

        if (fabsf(state->current_x) <= 0.04f) state->current_x = 0;
        if (fabsf(state->current_y) <= 0.04f) state->current_y = 0;
    } else {
        state->zclearflag1 = state->zclearflag2 = 0;
    }

    // 长时间无动作清零
    if (state->z_up_flag == 0) {
        state->zt++;
        if (state->zt >= 15 * config.ActuallongWindowSize) {
            state->only_zclear = 1;
            clear_function_point(state);
        }
    } else {
        state->zt = 0;
    }

    // XY轴清零条件
    if (fabsf(state->sensor_processed_xy) <= 0.04f &&
        fabsf(state->current_x) <= 0.04f &&
        fabsf(state->current_y) <= 0.04f &&
        fabsf(state->current_z) <= 0.25f) {
        state->xyt1++;
        if (state->xyt1 >= 5.0f * config.ActuallongWindowSize) {
            state->only_zclear = 1;
            clear_function_point(state);
        }
    } else {
        state->xyt1 = 0;
    }

    // 指尖传感器特殊清零条件
    if (strcmp(sensor_type, "SF-N20T10") == 0) {
        if ((fabsf(state->current_x) <= 0.01f && fabsf(state->current_y) <= 0.01f && fabsf(state->current_z) <= 1.5f) ||
            (fabsf(state->current_x) <= 0.01f && state->current_z <= 0.01f && fabsf(state->current_y) <= 1.0f) ||
            (fabsf(state->current_y) <= 0.01f && state->current_z <= 0.01f && fabsf(state->current_x) <= 1.0f)) {
            state->xyt5++;
            if (state->xyt5 >= 5 * config.ActuallongWindowSize) {
                state->only_zclear = 0;
                clear_function_point(state);
            }
        } else {
            state->xyt5 = 0;
        }

        if ((state->current_x == 0 || state->current_y == 0 || state->current_z == 0) &&
            (fabsf(state->current_x) <= 0.15f && fabsf(state->current_y) <= 0.15f && fabsf(state->current_z) <= 0.15f)) {
            state->xyt6++;
            if (state->xyt6 >= 5 * config.ActuallongWindowSize) {
                state->only_zclear = 0;
                clear_function_point(state);
            }
        } else {
            state->xyt6 = 0;
        }
    }

    // 指腹传感器特殊清零条件
    if (strcmp(sensor_type, "SP-N20T10") == 0) {
        if ((fabsf(state->current_x) <= 0.01f && fabsf(state->current_y) <= 0.01f && fabsf(state->current_z) <= 0.5f) ||
            (fabsf(state->current_x) <= 0.01f && state->current_z <= 0.01f && fabsf(state->current_y) <= 0.35f) ||
            (fabsf(state->current_y) <= 0.01f && state->current_z <= 0.01f && fabsf(state->current_x) <= 0.35f)) {
            state->xyt5++;
            if (state->xyt5 >= 5 * config.ActuallongWindowSize) {
                state->only_zclear = 0;
                clear_function_point(state);
            }
        } else {
            state->xyt5 = 0;
        }

        if ((state->current_x == 0 || state->current_y == 0 || state->current_z == 0) &&
            (fabsf(state->current_x) <= 0.15f && fabsf(state->current_y) <= 0.15f && fabsf(state->current_z) <= 0.15f)) {
            state->xyt6++;
            if (state->xyt6 >= 10 * config.ActuallongWindowSize) {
                state->only_zclear = 0;
                clear_function_point(state);
            }
        } else {
            state->xyt6 = 0;
        }
    }

    if (fabsf(state->current_x) < 0.1f) state->current_x = 0;
    if (fabsf(state->current_y) < 0.1f) state->current_y = 0;
    if (fabsf(state->current_z) < 0.1f) state->current_z = 0;
}

// 标准模式处理
static void process_standard_mode(sensor_processing_state_t* state) {
    const char* sensor_type = model_param;

    // 状态机
    if ((state->z_dev2 > 5) && (state->current_z > 0.0f)) {
        state->z_up_flag = -1;
        state->Current_CheckNum = 0;
    }
    if (((fabsf(state->x_dev2) > 30) || (fabsf(state->y_dev2) > 30)) && (state->z_up_flag == 0)) {
        state->z_up_flag = -2;
        state->Current_CheckNum = 0;
    }
    if (((fabsf(state->x_dev2) > 500) || (fabsf(state->y_dev2) > 500)) && (state->z_up_flag == -2)) {
        state->z_up_flag = -3;
        state->Current_CheckNum = 0;
    }
    if (((fabsf(state->x_dev2) > 2000) || (fabsf(state->y_dev2) > 2000)) && (state->z_up_flag == -3)) {
        state->z_up_flag = -4;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -2) && (state->z_up_flag == -1)) {
        state->z_up_flag = 1;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -15) && (state->z_up_flag == 1)) {
        state->z_up_flag = 2;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -50) && (state->z_up_flag != 3)) {
        state->z_up_flag = 3;
        state->Current_CheckNum = 0;
    }

    handle_z_up_states(state, sensor_type);

    // 标定缩放
    state->current_x = (state->current_x / (float)config.X_f) / config.X_T;
    state->current_y = (state->current_y / (float)config.Y_f) / config.Y_T;
    state->current_z = (state->current_z / (float)config.Z_f) / config.Z_T;

    // 指尖传感器额外缩放
//    if (strcmp(sensor_type, "SMR-N100T10") || strcmp(sensor_type, "SF-N20T10") == 0) {
//    	state->current_x *= 0.5f;
//    	state->current_y *= 0.5f;
//    	state->current_z *= 0.5f;
//    }

    state->sensor_processed_xy = state->current_x - state->current_y;

    update_still_detection(state);
    update_clear_conditions(state, sensor_type);
}

// 动态模式处理
static void process_dynamic_mode(sensor_processing_state_t* state) {
    const char* sensor_type = model_param;

    if ((state->z_dev2 > 0) && (state->current_z > 0)) {
        state->z_up_flag = -1;
        state->Current_CheckNum = 0;
    }
    if (state->z_dev2 < -10) {
        state->z_up_flag = 2;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -50) && (state->z_up_flag != 3)) {
        state->z_up_flag = 3;
        state->Current_CheckNum = 0;
    }

    if (state->z_up_flag == 2 || state->z_up_flag == 3) {
        float threshold = (state->z_up_flag == 2) ? 30.0f : 50.0f;
        float z_threshold = (state->z_up_flag == 2) ? 3.0f : 5.0f;
        if (fabsf(state->x_dev2) < 7 && fabsf(state->y_dev2) < 7 && fabsf(state->z_dev2) < 4) {
            process_data_check(state, threshold, threshold, z_threshold, 1);
        }
    }

    state->current_x = (state->current_x / (float)config.X_f) / config.X_T;
    state->current_y = (state->current_y / (float)config.Y_f) / config.Y_T;
    state->current_z = (state->current_z / (float)config.Z_f) / config.Z_T;
}

// 抗磁模式处理
static void process_antimagnetic_mode(sensor_processing_state_t* state) {
    const char* sensor_type = model_param;

    if ((state->z_dev2 > 5) && (state->current_z > 5) &&
        ((state->x_dev2 < -50) || (state->x_dev2 > 50) || (state->y_dev2 < -50) || (state->y_dev2 > 50)) &&
        ((fabsf(state->current_x) > 105) || (fabsf(state->current_y) > 105))) {
        state->z_up_flag = -1;
        state->Current_CheckNum = 0;
    }
    if (((fabsf(state->x_dev2) > 500) || (fabsf(state->y_dev2) > 500)) && (state->z_up_flag == 0)) {
        state->z_up_flag = -3;
        state->Current_CheckNum = 0;
    }
    if (((fabsf(state->x_dev2) > 2000) || (fabsf(state->y_dev2) > 2000)) && (state->z_up_flag == -3)) {
        state->z_up_flag = -4;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -2) && (state->z_up_flag == -1)) {
        state->z_up_flag = 1;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -15) && (state->z_up_flag == 1)) {
        state->z_up_flag = 2;
        state->Current_CheckNum = 0;
    }
    if ((state->z_dev2 < -50) && (state->z_up_flag != 3)) {
        state->z_up_flag = 3;
        state->Current_CheckNum = 0;
    }

    handle_z_up_states(state, sensor_type);

    state->current_x = (state->current_x / (float)config.X_f) / config.X_T;
    state->current_y = (state->current_y / (float)config.Y_f) / config.Y_T;
    state->current_z = (state->current_z / (float)config.Z_f) / config.Z_T;

    state->sensor_processed_xy = state->current_x - state->current_y;

    update_still_detection(state);
    update_clear_conditions(state, sensor_type);
}

// 模式分发（单通道）
static void process_sensor_by_mode_point(sensor_processing_state_t* state) {
    switch (working_mode) {
        case 1: process_standard_mode(state); break;
        case 2: process_dynamic_mode(state);  break;
        case 3: process_antimagnetic_mode(state); break;
        default:
            state->current_x = 1.0f;
            state->current_y = 2.0f;
            state->current_z = 3.0f;
            break;
    }
}

// 单通道清零函数（已在上面使用）
void clear_function_point(sensor_processing_state_t* state) {
    state->t = 0;
    state->xyt1 = state->xyt2 = state->xyt3 = 0;
    state->xyt4 = state->xyt5 = state->xyt6 = 0;
    state->zt = 0;
    state->zclearflag1 = state->zclearflag2 = 0;

    if (state->only_zclear == 0) {
        state->pre_x = 0.0f;
        state->pre_y = 0.0f;
        state->pre_z = 0.0f;
    } else {
        state->pre_z = 0.0f;
    }

    state->long_idx = 0;
    state->z_up_flag = 0;
    state->X_long_sum = state->Y_long_sum = state->Z_long_sum = 0.0;
    state->FluentWindowfirstflag = 0;
    state->Current_CheckNum = 0;
    state->xtotal = state->ytotal = state->ztotal = 0.0;
    state->smoothIndex = 0;
    state->size = 0;
    state->current_x = state->current_y = state->current_z = 0.0f;
    state->still_update_flag = 0;
    state->still_flag = 0;
    state->still_release_count = 0;
    memset(&state->still, 0, sizeof(state->still));
    state->stillgap.x0 = state->stillgap.y0 = state->stillgap.z0 = 0.0f;
    memset(state->xsmoothwindow, 0, sizeof(state->xsmoothwindow));
    memset(state->ysmoothwindow, 0, sizeof(state->ysmoothwindow));
    memset(state->zsmoothwindow, 0, sizeof(state->zsmoothwindow));
}

// 全通道清零（对外接口）
void clear_function_all(void) {
    for (int i = 0; i < ARRAY_PS; i++) {
        clear_function_point(&sensor_processing_state[i]);
    }
}

//static void update_sensor_status_point(float x, float y, float z, uint8_t *status) {
//    if (x >= 10.0f || x <= -10.0f ||
//        y >= 10.0f || y <= -10.0f ||
//        z >= 20.0f) {
//        *status = 1;
//    } else {
//        *status = 0;
//    }
//}

void update_sensor_status(sensor_processing_state_t* state, float* x, float* y, float* z) {
        int16_t out_x = 100 * state->current_x;
        int16_t out_y = 100 * state->current_y;
        int16_t out_z = 100 * state->current_z;

        state->current_x = out_x / 100.0f;
        state->current_y = out_y / 100.0f;
        state->current_z = out_z / 100.0f;

}

// ==================== 主数据处理函数（多通道） ====================
void Data_processing(SensorData *data_temp) {
    for (int point = 0; point < ARRAY_PS; point++) {
        sensor_processing_state_t* state = &sensor_processing_state[point];

        // 获取原始数据
        float raw_x, raw_y, raw_z;
        switch (point) {
            case 0:
                raw_x = data_temp->x0; raw_y = data_temp->y0; raw_z = data_temp->z0;
                break;
//            case 1:
//                raw_x = data_temp->x1; raw_y = data_temp->y1; raw_z = data_temp->z1;
//                break;
            default:
                continue;
        }

        // 预处理阶段（零点漂移采集）
        state->t++;

        if (state->t > 1 && state->t <= 10) {
            if (state->only_zclear) {
                state->pre_z += raw_z;
            } else {
                state->pre_x += raw_x;
                state->pre_y += raw_y;
                state->pre_z += raw_z;
            }
        } else if (state->t == 11) {
            if (state->only_zclear) {
                state->pre_z /= 9.0f;
            } else {
                state->pre_x /= 9.0f;
                state->pre_y /= 9.0f;
                state->pre_z /= 9.0f;
            }
        } else if (state->t > 11) {
        	state->t = 11;

            // 减去零点偏移
            float x = raw_x - state->pre_x;
            float y = raw_y - state->pre_y;
            float z = raw_z - state->pre_z;

            // 长窗口滑动平均及偏差计算
            update_sliding_window(state, x, y, z);

            // 保存当前值（供后续模式处理使用）
            state->current_x = x;
            state->current_y = y;
            state->current_z = z;

            // 模式处理（状态机、标定、静止检测、清零等）
            process_sensor_by_mode_point(state);

            // 非线性校准
            apply_sensor_calibration_point(&state->current_x, &state->current_y, &state->current_z, point);

            // 平滑滤波
            process_sensor_data_point(state, &state->current_x, &state->current_y, &state->current_z);

            update_sensor_status(state, &state->current_x, &state->current_y, &state->current_z);

            // 将处理后的值写回
            switch (point) {
                case 0:
					sensor_processed_data.x0 = state->current_x;
					sensor_processed_data.y0 = state->current_y;
					sensor_processed_data.z0 = state->current_z;
//                    update_sensor_status_point(state->current_x, state->current_y, state->current_z, &data_temp->status);
                    break;
//                case 1:
//					sensor_processed_data.x1 = state->current_x;
//					sensor_processed_data.y1 = state->current_y;
//					sensor_processed_data.z1 = state->current_z;
////                    update_sensor_status_point(state->current_x, state->current_y, state->current_z, &data_temp->status);
//                    break;
            }
        }
    }
}

// 辅助函数：trim（保留）
void trim(char *str) {
    char *end;
    while(isspace((unsigned char)*str) || iscntrl((unsigned char)*str)) str++;
    if(*str == 0) return;
    end = str + strlen(str) - 1;
    while(end > str && (isspace((unsigned char)*end) || iscntrl((unsigned char)*end))) end--;
    *(end+1) = 0;
}
