#ifndef INC_DATA_PROCESSING_H_
#define INC_DATA_PROCESSING_H_

#include "global.h"
#include "main.h"
#include "stm32h5xx_it.h"

// 优化后的配置
#define SHORT_WINDOW_SIZE           20      // 短窗口大小（暂未使用）
#define LONG_WINDOW_SIZE            100     // 长窗口大小
#define OPTIMIZED_SMOOTH_WINDOW_SIZE 50      // 滑动平滑滤波窗口大小（改为5，与single一致）
#define SMA_WINDOW_SIZE             5       // SMA窗口大小（暂未使用）
#define DATA_CHECK_SIZE             20      // 数据校验缓冲区大小（对应 ActualDATA_CheckSIZE）

// 优化后的数据结构
typedef struct {
    uint8_t only_zclear;
    int16_t t;
    uint8_t size;
    int8_t z_up_flag;
    float sensor_processed_xy;

    // 计数器优化为16位
    uint16_t zclearflag1, zclearflag2, zt;
    uint16_t xyt1, xyt2, xyt3, xyt4, xyt5, xyt6;

    // 窗口索引
    uint8_t smoothIndex;
    uint16_t LongwindowIndex;
    uint8_t Current_CheckNum;

    // 信号处理
    double xtotal, ytotal, ztotal;

    // 状态标志
    uint8_t FluentWindowfirstflag : 1;
    uint8_t still_update_flag : 1;
    uint16_t still_flag;
    uint8_t still_release_count;

    float pre_x;   // X轴零点偏移累加和
    float pre_y;   // Y轴零点偏移累加和
    float pre_z;   // Z轴零点偏移累加和

    // 处理数据
    SensorData still;
    SensorData stillgap;
    SensorData fitting_sensor_processed_data;
    SensorData last_sensor_processed_data;

    // SMA 滤波相关（暂未使用）
    float sma_x_buffer[SMA_WINDOW_SIZE];
    float sma_y_buffer[SMA_WINDOW_SIZE];
    float sma_z_buffer[SMA_WINDOW_SIZE];
    uint16_t sma_index;
    float sma_x_sum, sma_y_sum, sma_z_sum;
    uint8_t sma_initialized;

    // 双窗口基线滤波器（single算法仅使用长窗口部分）
    // 短窗口（保留，但single未使用）
    float X_shortwindow[SHORT_WINDOW_SIZE];
    float Y_shortwindow[SHORT_WINDOW_SIZE];
    float Z_shortwindow[SHORT_WINDOW_SIZE];
    uint16_t short_idx;
    double X_short_sum, Y_short_sum, Z_short_sum;
    float X_short_mean, Y_short_mean, Z_short_mean;
    uint8_t short_initialized;

    // 长窗口
    float X_longwindow[LONG_WINDOW_SIZE];
    float Y_longwindow[LONG_WINDOW_SIZE];
    float Z_longwindow[LONG_WINDOW_SIZE];
    uint16_t long_idx;
    double X_long_sum, Y_long_sum, Z_long_sum;
    float X_long_mean, Y_long_mean, Z_long_mean;
    uint8_t long_initialized;

    // 偏差值
    float x_dev1, y_dev1, z_dev1;   // 对短窗口偏差（未使用）
    float x_dev2, y_dev2, z_dev2;   // 对长窗口偏差（single使用）

    // 数据有效性保持（single未使用，但保留）
    float last_valid_x, last_valid_y, last_valid_z;
    float last_valid_x2, last_valid_y2, last_valid_z2;
    uint8_t hold2_primed;

    // 阈值计数（single未使用，保留）
    uint16_t vaild_flag_x, vaild_flag_y, vaild_flag_z;
    uint16_t invaild_flag_x, invaild_flag_y, invaild_flag_z;
    uint16_t clear_flag1, clear_flag2, clear_flag5, clear_flag6;

    // 阈值参数（single未使用，保留）
    uint8_t x_flag_thrshold1, y_flag_thrshold1, z_flag_thrshold1;
    uint8_t x_flag_thrshold2, y_flag_thrshold2, z_flag_thrshold2;
    float min_x, max_x, min_y, max_y, min_z, max_z;
    float x_mesure_range, y_mesure_range, z_mesure_range;

    // 超量程标记
    uint8_t overload_flag;

    // 环形缓冲平滑（窗口大小由 OPTIMIZED_SMOOTH_WINDOW_SIZE 决定）
    float xsmoothwindow[OPTIMIZED_SMOOTH_WINDOW_SIZE];
    float ysmoothwindow[OPTIMIZED_SMOOTH_WINDOW_SIZE];
    float zsmoothwindow[OPTIMIZED_SMOOTH_WINDOW_SIZE];

    // 输出标志
    uint16_t output_flag;

    // ========== 新增成员（single算法需要） ==========
    // 数据校验缓冲区（用于 process_data_check）
    float XDATA_Check[DATA_CHECK_SIZE];
    float YDATA_Check[DATA_CHECK_SIZE];
    float ZDATA_Check[DATA_CHECK_SIZE];

    // 当前处理中的数值（避免直接修改原始数据）
    float current_x;
    float current_y;
    float current_z;

} sensor_processing_state_t;

// 函数声明
void Data_processing(SensorData *data_temp);
void clear_function_point(sensor_processing_state_t* state);
void clear_function_all(void);
void trim(char *str);
void update_sliding_window(sensor_processing_state_t* state, float x, float y, float z);
void process_sensor_by_mode(sensor_processing_state_t* state);
void apply_sensor_calibration_point(float* x, float* y, float* z, uint8_t sensor_idx);
void process_sensor_data_point(sensor_processing_state_t* state, float* x, float* y, float* z);
void sensor_processing_state_init_point(sensor_processing_state_t* state);
void sensor_processing_state_init_all(void);
void update_sensor_status(sensor_processing_state_t* state, float* x, float* y, float* z);

// 外部变量声明
extern uint8_t start_flag;
extern uint8_t delay_f;
extern sensor_processing_state_t sensor_processing_state[ARRAY_PS];

#endif /* INC_DATA_PROCESSING_H_ */
