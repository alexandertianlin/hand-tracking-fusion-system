#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdint.h>
#include <stddef.h>
//according to points to fix parameters starting
#define MLX90393_I2C_ADDRESS_1 (0x0C << 1)	//left shift due to MLX I2C addr+[W/R]	A1A0=00
#define MLX90393_I2C_ADDRESS_3 (0x0D << 1)	//left shift due to MLX I2C addr+[W/R]	A1A0=01
#define MLX90393_I2C_ADDRESS_2 (0x0E << 1)	//left shift due to MLX I2C addr+[W/R]	A1A0=10
#define MLX90393_I2C_ADDRESS_4 (0x0F << 1)	//left shift due to MLX I2C addr+[W/R]	A1A0=11

#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    // 协议帧头
    uint8_t  frame_header0;   // 0xB5
    uint8_t  frame_header1;   // 0xA5
    uint8_t  frame_header2;   // 0x55
    uint16_t frame_length;    // 0x0023 (35)

    // 基础信息
    uint8_t  model_id;
    uint8_t  id;
    uint8_t  status;

    // ✅【关键】和原版一致：用 float 存储原始IMU数据（赋值不会变0）
    float    quat_w;
    float    quat_x;
    float    quat_y;
    float    quat_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;

    // 力场数据
    float    x0;
    float    y0;
    float    z0;

    // 校验
    uint8_t  crc;
} SensorData;
#pragma pack(pop) // 恢复默认对齐

typedef struct {
	uint32_t target_interval_ms;  // 目标间隔(ms)
	uint32_t actual_interval_ms;  // 实际间隔(ms)
	uint32_t last_send_time;      // 上次发送时间
} send_controller_t;

#define ARRAY_PS 1

#define MLX90393_TEMP_READING		1 // 讀取 mlx溫度數據
#define MLX90393_TEMP_OUTPUT		0 // 輸出 mlx溫度數據 float tcmp_c
#define PALM_IMU_ENABLE				1 // 0: disable IMU I2C init/read to avoid MLX bus interference
//according to points to fix parameters ending
#define MAX_BOARDS 5
#define ACK_BYTE 0xB5//
#define SLAVE_ACK_STATUS 0x5B//
#define START_BYTE 0xB4//
#define CMD_STATUS_BYTE 0xAA//
#define HOST_ACK_BYTEX 0xFF
#define HOST_ACK_BYTE0 0xEE
#define RESET_BYTE 0xF0
#define MODEL_READING_CMD  0xB0
#define START_READING_CMD1 0xB1
#define START_READING_CMD2 0xB2
#define START_READING_CMD3 0xB3
#define C_Serial_READING_CMD 0xCC
#define D_Serial_READING_CMD 0xDD

#define Interval_CMD0 0xA0
#define Interval_CMD1 0xA1
#define Interval_CMD2 0xA2
#define Interval_CMD3 0xA3
#define Interval_CMD4 0xA4
#define Interval_CMD5 0xA5

typedef struct __attribute__((packed)){
    uint8_t MCU_UPGRADE_FLAG;
    uint8_t BOOTLOADER_VERSION;
    uint8_t SOFTWARE_VERSION1;
    uint8_t SOFTWARE_VERSION2;

    uint8_t CODE_ZONE;
    uint8_t HOST_RESET;
    uint8_t HARDWARE_VERSION1;
    uint8_t HARDWARE_VERSION2;
} STM_HOST_DATA;

extern int direction;
extern double K0_0;
extern double K1_0;
extern double K2_0;
extern double K3_0;
extern double K4_0;
extern double K5_0;
extern double K6_0;
extern double K7_0;
extern double K8_0;
extern double K9_0;

extern double a_x_0;
extern double b_x_0;
extern double c_x_0;

extern double a_y_0;
extern double b_y_0;
extern double c_y_0;

extern double a_z_0;
extern double b_z_0;
extern double c_z_0;

extern double K0_1;
extern double K1_1;
extern double K2_1;
extern double K3_1;
extern double K4_1;
extern double K5_1;
extern double K6_1;
extern double K7_1;
extern double K8_1;
extern double K9_1;

extern double a_x_1;
extern double b_x_1;
extern double c_x_1;

extern double a_y_1;
extern double b_y_1;
extern double c_y_1;

extern double a_z_1;
extern double b_z_1;
extern double c_z_1;

extern double K0_2;
extern double K1_2;
extern double K2_2;
extern double K3_2;
extern double K4_2;
extern double K5_2;
extern double K6_2;
extern double K7_2;
extern double K8_2;
extern double K9_2;

extern double a_x_2;
extern double b_x_2;
extern double c_x_2;

extern double a_y_2;
extern double b_y_2;
extern double c_y_2;

extern double a_z_2;
extern double b_z_2;
extern double c_z_2;

extern double K0_3;
extern double K1_3;
extern double K2_3;
extern double K3_3;
extern double K4_3;
extern double K5_3;
extern double K6_3;
extern double K7_3;
extern double K8_3;
extern double K9_3;

extern double a_x_3;
extern double b_x_3;
extern double c_x_3;

extern double a_y_3;
extern double b_y_3;
extern double c_y_3;

extern double a_z_3;
extern double b_z_3;
extern double c_z_3;

extern const char model_param[16];
// define struct for sensor data and id



#endif
