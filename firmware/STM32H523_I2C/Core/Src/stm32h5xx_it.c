/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "fsm.h"
#include "data_processing.h"
#include "app/fingertip_runtime.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
void read_mlx90393_burst(void);
// Global flag to signal main loop
volatile uint16_t Count = 0;  //RX Count
volatile uint8_t Head_flag = 0;//flag for effective data packet frame
volatile uint8_t Cali_flag = 0;//flag for calibration data
volatile uint16_t Receive_len = 0;
uint16_t packet_buffer_len = 3+152*ARRAY_PS;     // calibration format=0x5A 【0xFF 0xFF 0xFF 0xFF】【】 0xA5
uint16_t rx_buffer_len = 3+152*ARRAY_PS+25;//Max RX buffer and 25 is margin
 uint8_t calibration_uart_rx_buffer[3+152*ARRAY_PS];
 uint8_t dynamic_uart_rx_buffer[3+152*ARRAY_PS+25];
volatile uint8_t g_data_read_flag = 0;
volatile uint8_t g_coil_read_flag = 0;	//flag for mlx90393 timer3 225hz
// Global variables for system data

SensorData system[MAX_BOARDS];
uint8_t system_count = 1; // system[0] is always local
uint8_t uart_rx_buffer;
uint8_t uart_rx_buffer_arr[sizeof(system)];
uint8_t uart_rx_buffer_index = 0;
uint8_t uart_tx_buffer[sizeof(SensorData) + 2];
uint8_t master_rx_buffer;
uint8_t master_tx_buffer[3]; // 3 bytes cmd from master send to next slave
uint8_t SensorID = 1;
uint32_t sensor_last_update_tick[MAX_BOARDS];

send_controller_t tx_ctrl = {
    .target_interval_ms = 10,  // 1000Hz 默认
    .actual_interval_ms = 10,
    .last_send_time = 0,
};

extern I2C_HandleTypeDef hi2c1;
extern uint8_t working_mode;
extern fingertip_runtime_t g_fingertip_runtime;

#define MLX90393_TEMP_COMPENSATE_X 1
#define MLX90393_TEMP_COMPENSATE_Y 1
#define MLX90393_TEMP_COMPENSATE_Z 1

#define MLX90393_TEMP_MIN_C          0.0f
#define MLX90393_TEMP_MAX_C         80.0f
#define MLX90393_TEMP_MAX_STEP_C     0.2f
#define MLX90393_TEMP_FILTER_ALPHA   0.02f

#define MLX90393_TEMP_X_A       -0.241212f
#define MLX90393_TEMP_X_B       -2.708378f
#define MLX90393_TEMP_X_C     3477.265595f
#define MLX90393_TEMP_X_BASE  3042.1812f

#define MLX90393_TEMP_Y_A        0.073475f
#define MLX90393_TEMP_Y_B       13.393507f
#define MLX90393_TEMP_Y_C    -3176.880713f
#define MLX90393_TEMP_Y_BASE -2576.6713f

#define MLX90393_TEMP_Z_A       -0.179165f
#define MLX90393_TEMP_Z_B        0.944146f
#define MLX90393_TEMP_Z_C     1966.384073f
#define MLX90393_TEMP_Z_BASE  1753.2041f

static float mlx90393_temperature_fit(float temperature_c,
                                      float a,
                                      float b,
                                      float c)
{
	return (a * temperature_c * temperature_c) + (b * temperature_c) + c;
}

static float mlx90393_filter_temperature(float temperature_c)
{
	static uint8_t initialized = 0;
	static float filtered_temperature = 25.0f;
	float limited_temperature;

	if ((temperature_c < MLX90393_TEMP_MIN_C) ||
	    (temperature_c > MLX90393_TEMP_MAX_C)) {
		return filtered_temperature;
	}

	if (!initialized) {
		filtered_temperature = temperature_c;
		initialized = 1;
		return filtered_temperature;
	}

	limited_temperature = temperature_c;
	if ((limited_temperature - filtered_temperature) > MLX90393_TEMP_MAX_STEP_C) {
		limited_temperature = filtered_temperature + MLX90393_TEMP_MAX_STEP_C;
	} else if ((filtered_temperature - limited_temperature) > MLX90393_TEMP_MAX_STEP_C) {
		limited_temperature = filtered_temperature - MLX90393_TEMP_MAX_STEP_C;
	}

	filtered_temperature += MLX90393_TEMP_FILTER_ALPHA *
	                        (limited_temperature - filtered_temperature);
	return filtered_temperature;
}

static void mlx90393_apply_temperature_compensation(SensorData *data,
                                                    float temperature_c)
{
	if (data == NULL) {
		return;
	}

#if MLX90393_TEMP_COMPENSATE_X
	data->x0 -= (mlx90393_temperature_fit(temperature_c,
	                                      MLX90393_TEMP_X_A,
	                                      MLX90393_TEMP_X_B,
	                                      MLX90393_TEMP_X_C) -
	             MLX90393_TEMP_X_BASE);
#endif

#if MLX90393_TEMP_COMPENSATE_Y
	data->y0 -= (mlx90393_temperature_fit(temperature_c,
	                                      MLX90393_TEMP_Y_A,
	                                      MLX90393_TEMP_Y_B,
	                                      MLX90393_TEMP_Y_C) -
	             MLX90393_TEMP_Y_BASE);
#endif

#if MLX90393_TEMP_COMPENSATE_Z
	data->z0 -= (mlx90393_temperature_fit(temperature_c,
	                                      MLX90393_TEMP_Z_A,
	                                      MLX90393_TEMP_Z_B,
	                                      MLX90393_TEMP_Z_C) -
	             MLX90393_TEMP_Z_BASE);
#endif
}
// Static variables for interrupt-based multi-packet transmission
#if MLX90393_TEMP_OUTPUT
	static uint8_t tx_packet_buf[20];//TCMP
#else
	static uint8_t tx_packet_buf[1 + sizeof(SensorData) + 1];
#endif
static uint8_t tx_packet_index = 0;

static slave_rx_state_t slave_rx_state = RX_WAIT_START;
static slave_rx_mode_t slave_rx_mode = RX_TBD;
static slave_tx_mode_t slave_tx_mode = TX_IDLE;
static main_loop_state_t main_loop_state = ACK_MODE;
const char D_Serial[] = "\r\nD-25-xx-xx-xx-xx-xx\n";
const char C_Serial[] = "\r\nC-26-xx-xx-xx-xx-xx\n";
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

double K0_1;
double K1_1;
double K2_1;
double K3_1;
double K4_1;
double K5_1;
double K6_1;
double K7_1;
double K8_1;
double K9_1;

double a_x_1;
double b_x_1;
double c_x_1;

double a_y_1;
double b_y_1;
double c_y_1;

double a_z_1;
double b_z_1;
double c_z_1;

double K0_2;
double K1_2;
double K2_2;
double K3_2;
double K4_2;
double K5_2;
double K6_2;
double K7_2;
double K8_2;
double K9_2;

double a_x_2;
double b_x_2;
double c_x_2;

double a_y_2;
double b_y_2;
double c_y_2;

double a_z_2;
double b_z_2;
double c_z_2;

double K0_3;
double K1_3;
double K2_3;
double K3_3;
double K4_3;
double K5_3;
double K6_3;
double K7_3;
double K8_3;
double K9_3;

double a_x_3;
double b_x_3;
double c_x_3;

double a_y_3;
double b_y_3;
double c_y_3;

double a_z_3;
double b_z_3;
double c_z_3;


int direction = 1;
uint16_t tcmp_raw;
float tcmp_c;
const char model_param[16] = "SP-N20T10";
//uint32_t reg_holder;
/* USER CODE END TD */
extern void Data_processing (SensorData *data_temp);
extern SensorData sensor_processed_data;
SensorData data_temp = {0};
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void transmit_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void init_system_protocol(void)
{
    for(int i=0; i<MAX_BOARDS; i++)
    {
        system[i].frame_header0 = 0xB5;
        system[i].frame_header1 = 0xA5;
        system[i].frame_header2 = 0x55;
        system[i].frame_length = 0x0023; // 小端35
    }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
// Helper: Calculate CRC (XOR of payload)
// CRC Calculation does not include the start byte!
static uint8_t calc_crc(const uint8_t* data, uint16_t len) {
	uint8_t crc = 0;
	for (uint16_t i = 0; i < len; ++i) {
		crc ^= data[i];
	}
	return crc;
}

//static void transmit_sensor_packet_IT0(const SensorData* data);
static void transmit_sensor_packet_ITX(const SensorData* data);
void start_transmission(void);
uint8_t should_send_now(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
		while (HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1) != HAL_OK) {}
	}
	HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	if(HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY) {
		while (HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1) != HAL_OK) {}
	}
	HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles Instruction cache global interrupt.
  */
void ICACHE_IRQHandler(void)
{
  /* USER CODE BEGIN ICACHE_IRQn 0 */

  /* USER CODE END ICACHE_IRQn 0 */
  HAL_ICACHE_IRQHandler();
  /* USER CODE BEGIN ICACHE_IRQn 1 */

  /* USER CODE END ICACHE_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM2) {
		static uint8_t protocol_inited = 0;
		if(!protocol_inited){
			init_system_protocol();
			protocol_inited = 1;
		}

		if (main_loop_state != ACK_MODE) {
			__atomic_store_n(&g_data_read_flag, 1, __ATOMIC_SEQ_CST);
		} else if (main_loop_state == ACK_MODE) {
			SensorData ack_packet = {0};
            // 🔥 修复：初始化ACK包必备参数
			ack_packet.model_id = 0x14;
			ack_packet.id = 0;
			transmit_sensor_packet_ITX(&ack_packet);
			slave_tx_mode = TX_ACK_MODE;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {

	if (huart->Instance == USART2) {
		uint8_t byte = master_rx_buffer;
//flash parameters index start
		Count++;
		dynamic_uart_rx_buffer[Count-1] = byte;
		if((byte == 0x5A)&&(Head_flag == 0)) {
			main_loop_state = FLASH_MODE;
			Head_flag = 1;
			Receive_len = 1;
		} else if (Head_flag == 1) {
			Receive_len++;
			if(byte == 0xA5) {
				for (uint8_t i = 0; i < rx_buffer_len - packet_buffer_len; i++) {
					if(dynamic_uart_rx_buffer[i] == 0x5A) {
						for (uint16_t j = 0; j < packet_buffer_len; j++) {
							calibration_uart_rx_buffer[j] = dynamic_uart_rx_buffer[i+j];
						}
						if (calibration_uart_rx_buffer[packet_buffer_len-1] == 0xA5) {
							Cali_flag = 1;
							memset(dynamic_uart_rx_buffer,0,sizeof(dynamic_uart_rx_buffer));
							Count = 0;
							Head_flag = 0;
						} else {
							Head_flag = 1;
						}
					}
				}
			}
		} else {
			Receive_len++;
			if(Receive_len > packet_buffer_len) {
				Receive_len = 0;
				Head_flag = 0;
				memset(dynamic_uart_rx_buffer,0,sizeof(dynamic_uart_rx_buffer));
				Count = 0;
			}
		}
		if(Count >= rx_buffer_len) {
			memset(dynamic_uart_rx_buffer,0,sizeof(dynamic_uart_rx_buffer));
			Count = 0;
		}
//flash parameters index end

		// Single byte CMD implementation
		if ((byte == RESET_BYTE)&&(main_loop_state != FLASH_MODE)) {
			if (main_loop_state == READING_MODE) {
				main_loop_state = CMD_MODE;
				slave_tx_mode = TX_IDLE;
				slave_rx_state = RX_WAIT_START;
				slave_rx_mode = RX_TBD;
			}
			system_count = 1;
			memset(system, 0, sizeof(system));
			static const uint8_t reset_cmd = RESET_BYTE;
			HAL_UART_Transmit_IT(&huart1, &reset_cmd, 1);
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == START_READING_CMD1)&&(main_loop_state != FLASH_MODE)) {
			main_loop_state = READING_MODE;
			slave_tx_mode = TX_READING_MODE;
			read_mlx90393_burst();
			start_transmission();
			system_count = 1;
			memset(system, 0, sizeof(system));
			working_mode = 1;
			for (int i = 0; i < ARRAY_PS; i++) {
				sensor_processing_state[i].only_zclear = 0;
			}
			clear_function_all();
			uint8_t cmd_byte = START_READING_CMD1;
			HAL_UART_Transmit_IT(&huart1, &cmd_byte, 1);
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == START_READING_CMD2)&&(main_loop_state != FLASH_MODE)) {
			main_loop_state = READING_MODE;
			slave_tx_mode = TX_READING_MODE;
			read_mlx90393_burst();
			start_transmission();
			system_count = 1;
			memset(system, 0, sizeof(system));
			working_mode = 2;
			for (int i = 0; i < ARRAY_PS; i++) {
				sensor_processing_state[i].only_zclear = 0;
			}
			clear_function_all();
			uint8_t cmd_byte = START_READING_CMD2;
			HAL_UART_Transmit_IT(&huart1, &cmd_byte, 1);
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == START_READING_CMD3)&&(main_loop_state != FLASH_MODE)) {
			main_loop_state = READING_MODE;
			slave_tx_mode = TX_READING_MODE;
			read_mlx90393_burst();
			start_transmission();
			system_count = 1;
			memset(system, 0, sizeof(system));
			working_mode = 3;
			for (int i = 0; i < ARRAY_PS; i++) {
				sensor_processing_state[i].only_zclear = 0;
			}
			clear_function_all();
			uint8_t cmd_byte = START_READING_CMD3;
			HAL_UART_Transmit_IT(&huart1, &cmd_byte, 1);
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD0) && (main_loop_state == CMD_MODE)) {
			if(SensorID == 1){ tx_ctrl.target_interval_ms = 0; tx_ctrl.actual_interval_ms = 0;}
			else if(SensorID == 0){ tx_ctrl.target_interval_ms = 1; tx_ctrl.actual_interval_ms = 1;}
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD1) && (main_loop_state == CMD_MODE)) {
			tx_ctrl.target_interval_ms = 1; tx_ctrl.actual_interval_ms = 1;
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD2) && (main_loop_state == CMD_MODE)) {
			tx_ctrl.target_interval_ms = 2; tx_ctrl.actual_interval_ms = 2;
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD3) && (main_loop_state == CMD_MODE)) {
			tx_ctrl.target_interval_ms = 10; tx_ctrl.actual_interval_ms = 10;
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD4) && (main_loop_state == CMD_MODE)) {
			tx_ctrl.target_interval_ms = 100; tx_ctrl.actual_interval_ms = 100;
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == Interval_CMD5) && (main_loop_state == CMD_MODE)) {
			tx_ctrl.target_interval_ms = 0xFFFFFFFF; tx_ctrl.actual_interval_ms = 0xFFFFFFFF;
			main_loop_state = READING_MODE; slave_tx_mode = TX_READING_MODE;
			main_loop_state = CMD_MODE; slave_tx_mode = TX_IDLE;
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}
		else if ((byte == HOST_ACK_BYTEX)&&(main_loop_state != FLASH_MODE)) {
			if(main_loop_state == ACK_MODE){
				system[0].id = 0; system[0].status = 0;
				system[0].x0 = 0; system[0].y0 = 0; system[0].z0 = 0;
				SensorID = 1; tx_ctrl.target_interval_ms = 1; tx_ctrl.actual_interval_ms = 1;
				main_loop_state = CMD_MODE;
			}
			uint8_t cmd_byte = HOST_ACK_BYTEX;
			HAL_UART_Transmit_IT(&huart1, &cmd_byte, 1);
			// 重新开启接收
			HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
			return;
		}

		// 重新开启接收
		HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);
	}

	// ==================================================================
	// 下半部分：接收下游级联 (USART1) -> 核心逆向解包过程
	// ==================================================================
	if (huart->Instance == USART1) {
			static uint8_t rx_step = 0;
			static uint16_t rx_cnt = 0;

	        // 【核心修改】动态宏定义期望的包长度
	#if MLX90393_TEMP_OUTPUT
	        #define EXPECTED_LEN 39
	#else
	        #define EXPECTED_LEN 35
	#endif

			static uint8_t rx_raw_buf[EXPECTED_LEN];
			uint8_t byte = uart_rx_buffer;

			switch (rx_step) {
				case 0: // 0xB5
					if (byte == 0xB5) { rx_raw_buf[0] = byte; rx_step = 1; } break;
				case 1: // 0xA5
					if (byte == 0xA5) { rx_raw_buf[1] = byte; rx_step = 2; } else { rx_step = 0; } break;
				case 2: // 0x55
					if (byte == 0x55) { rx_raw_buf[2] = byte; rx_cnt = 3; rx_step = 3; } else { rx_step = 0; } break;
				case 3:
					rx_raw_buf[rx_cnt++] = byte;
					if (rx_cnt >= EXPECTED_LEN) {
						uint8_t computed_crc = 0;
	                    // CRC 只异或校验位之前的所有数据
						for (uint8_t i = 0; i < EXPECTED_LEN - 1; i++) { computed_crc ^= rx_raw_buf[i]; }

						if (byte == computed_crc) {
							uint8_t downstream_id = rx_raw_buf[6];
							// ✅ 真正的ID+1逻辑：本地ID = 下游ID + 1
							uint8_t local_id = downstream_id + 1;

							if (local_id < MAX_BOARDS) {
								// 禁用无关中断，防止竞态
								NVIC_DisableIRQ(TIM2_IRQn);
								NVIC_DisableIRQ(TIM3_IRQn);

								// 1. 提取基础信息
								system[local_id].model_id = rx_raw_buf[5];
								system[local_id].id = local_id; // ✅ 使用计算出的本地ID
								system[local_id].status = rx_raw_buf[7];

								// 2. 提取并解压 IMU 四元数 (int16 转回 float)
								int16_t temp_i16;
								memcpy(&temp_i16, &rx_raw_buf[8], 2);  system[local_id].quat_w = (float)temp_i16 / PALM_QUAT_SCALE;
								memcpy(&temp_i16, &rx_raw_buf[10], 2); system[local_id].quat_x = (float)temp_i16 / PALM_QUAT_SCALE;
								memcpy(&temp_i16, &rx_raw_buf[12], 2); system[local_id].quat_y = (float)temp_i16 / PALM_QUAT_SCALE;
								memcpy(&temp_i16, &rx_raw_buf[14], 2); system[local_id].quat_z = (float)temp_i16 / PALM_QUAT_SCALE;

								// 3. 提取并解压 IMU 加速度 (int16 转回 float)
								memcpy(&temp_i16, &rx_raw_buf[16], 2); system[local_id].accel_x = (float)temp_i16 / PALM_RAW_ACCEL_MG_SCALE;
								memcpy(&temp_i16, &rx_raw_buf[18], 2); system[local_id].accel_y = (float)temp_i16 / PALM_RAW_ACCEL_MG_SCALE;
								memcpy(&temp_i16, &rx_raw_buf[20], 2); system[local_id].accel_z = (float)temp_i16 / PALM_RAW_ACCEL_MG_SCALE;

								// 4. 提取 MLX 力矩 (直接内存拷贝到 float)
								memcpy(&system[local_id].x0, &rx_raw_buf[22], 4);
								memcpy(&system[local_id].y0, &rx_raw_buf[26], 4);
								memcpy(&system[local_id].z0, &rx_raw_buf[30], 4);

								// 5. 只有当新来的 local_id 拓展了系统规模时，才更新 system_count！
								uint8_t current_count = __atomic_load_n(&system_count, __ATOMIC_SEQ_CST);
								if ((local_id + 1) > current_count) {
								    __atomic_store_n(&system_count, local_id + 1, __ATOMIC_SEQ_CST);
								}

								// 更新最后更新时间
								sensor_last_update_tick[local_id] = HAL_GetTick();

								// 重新启用中断
								NVIC_EnableIRQ(TIM3_IRQn);
								NVIC_EnableIRQ(TIM2_IRQn);
							}
						}
						rx_step = 0;
					}
					break;
			}
			// 重新开启接收
			HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1);
	}
}

// Helper: Read MLX90393 burst data into system[0]
void read_mlx90393_burst(void) {
	  if (ARRAY_PS ==1)
	  {
#if MLX90393_TEMP_READING
		  //Enable temperature reading
		  uint8_t cmd = 0x4F;
		  uint8_t data[9];
		  HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		  HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, data, 9, HAL_MAX_DELAY);

		  data_temp.status = data[0];
		  tcmp_raw = (data[1] << 8 | data[2]);
		  data_temp.x0 = direction * (int16_t)(data[3] << 8 | data[4]);
		  data_temp.y0 = direction * (int16_t)(data[5] << 8 | data[6]);
		  data_temp.z0 = direction * (int16_t)(data[7] << 8 | data[8]);

		  tcmp_c = 25.0f + ((float)tcmp_raw - 46244.0f)/45.2f;
		  tcmp_c = mlx90393_filter_temperature(tcmp_c);

			if (data_temp.z0 < 0) {
				data_temp.z0 = (data_temp.z0 + 65536)/10;
			}
			else if (data_temp.z0 >= 0) {
				data_temp.z0 = (data_temp.z0)/10;
			}

			mlx90393_apply_temperature_compensation(&data_temp, tcmp_c);
#else
		  uint8_t cmd = 0x4E;
		  uint8_t data[7];
		  //I2C addr1 A1A0=00
		  HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		  HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, data, 7, HAL_MAX_DELAY);

		  //SensorData data_temp;
		  data_temp.x0 = direction * (int16_t)(data[1] << 8 | data[2]);
		  data_temp.y0 = direction * (int16_t)(data[3] << 8 | data[4]);
		  data_temp.z0 = direction * (int16_t)(data[5] << 8 | data[6]);

			if (data_temp.z0 < 0) {
				data_temp.z0 = (data_temp.z0 + 65536)/10;
			}
			else if (data_temp.z0 >= 0) {
				data_temp.z0 = (data_temp.z0)/10;
			}
#endif

		  Data_processing(&data_temp);

		  // ✅ 只更新MLX相关字段，不覆盖IMU数据！
		  system[0].x0 = sensor_processed_data.x0;
		  system[0].y0 = sensor_processed_data.y0;
		  system[0].z0 = sensor_processed_data.z0;
		  // 如果有其他MLX字段，在这里添加
		  // system[0].x1 = sensor_processed_data.x1;
		  // system[0].y1 = sensor_processed_data.y1;
		  // system[0].z1 = sensor_processed_data.z1;
	  }

	sensor_last_update_tick[0] = HAL_GetTick();
}

// 对齐你的标准：小端写入int16
static void palm_protocol_put_i16_le(uint8_t *buf, int16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

// 对齐你的标准：int16钳位
static int16_t palm_protocol_clamp_to_i16(float val)
{
    if (val > INT16_MAX) return INT16_MAX;
    if (val < INT16_MIN) return INT16_MIN;
    return (int16_t)val;
}

// 你的标准缩放系数
#define PALM_QUAT_SCALE         10000.0f
#define PALM_RAW_ACCEL_MG_SCALE 1000.0f


static void transmit_sensor_packet_ITX(const SensorData* data)
{
    if (slave_tx_mode == TX_IDLE) return;

    // 🛡️ 内存保护：必须加 static！保证函数结束后，这35字节不会被销毁！
    static uint8_t frame[35];
    memset(frame, 0, 35); // 每次使用前清零

    // 💥 修复：声明并初始化偏移量指针
    uint8_t payload_ptr = 8;

    // 1. 帧头（完全对齐）
    frame[0] = 0xB5;
    frame[1] = 0xA5;
    frame[2] = 0x55;

    // 2. 长度占位（最后回填）
    palm_protocol_put_i16_le(&frame[3], 0);

    // 3. 设备信息（严格对齐偏移5/6/7）
    frame[5] = data->model_id;
    frame[6] = data->id;         // node_id = 传感器ID
    frame[7] = data->status;

    // ====================== IMU 四元数 ======================
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->quat_w * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->quat_x * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->quat_y * PALM_QUAT_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->quat_z * PALM_QUAT_SCALE));
    payload_ptr += 2;

    // ====================== IMU 加速度 ======================
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->accel_x * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->accel_y * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;
    palm_protocol_put_i16_le(&frame[payload_ptr],
        palm_protocol_clamp_to_i16(data->accel_z * PALM_RAW_ACCEL_MG_SCALE));
    payload_ptr += 2;

    // ====================== 力数据 (直接拷贝 12 字节) ======================
    memcpy(&frame[payload_ptr], &data->x0, 4);
    payload_ptr += 4;
    memcpy(&frame[payload_ptr], &data->y0, 4);
    payload_ptr += 4;
    memcpy(&frame[payload_ptr], &data->z0, 4);
    payload_ptr += 4;

    // ====================== 长度回填 ======================
    uint16_t total_len = payload_ptr + 1;  // +CRC字节
    palm_protocol_put_i16_le(&frame[3], total_len);

    // ====================== CRC 计算 ======================
    uint8_t crc = 0;
    for (uint8_t i = 0; i < payload_ptr; i++)
    {
        crc ^= frame[i];
    }
    frame[payload_ptr] = crc;

    // 发送
    HAL_UART_Transmit_IT(&huart2, frame, 35);
}

//static void transmit_sensor_packet_IT0(const SensorData* data) {
//	if(HAL_GetTick() - sensor_last_update_tick[data->id] > 100 && slave_tx_mode == TX_READING_MODE) {
//		// sensor didn't update in 500ms, ignore this packet
//
//		system_count = data->id;
//		tx_packet_index = 0;
//		slave_tx_mode = TX_READING_MODE;
//		transmit_sensor_packet_ITX(&system[0]);
//		return;
//	}
//
//	#define TX_BUF_SIZE 50
//	static char tx_text_buf[TX_BUF_SIZE];
//	memset(tx_text_buf, 0, TX_BUF_SIZE); // 清零缓冲区
//	int len = 0;
//
//	switch (slave_tx_mode) {
//		case TX_ACK_MODE:
//			tx_packet_buf[0] = ACK_BYTE; // add start byte for ACK Mode
//			break;
//		case TX_CMD_MODE:
//			tx_packet_buf[0] = START_BYTE; // add start byte for CMD Mode
//			break;
//		case TX_READING_MODE:
//			if (data->status == SLAVE_ACK_STATUS) // this message is ACK from other board
//				tx_packet_buf[0] = ACK_BYTE;	  // add start byte for ACK Mode
//			else{
////				len = snprintf(tx_text_buf, TX_BUF_SIZE-1, "d%d %.2f %.2f %.2f %d\r\n",
////							  data->id,
////							  data->x,
////							  data->y,
////							  data->z,
////							  data->status);
//			}
//			break;
//		case TX_IDLE: // shld never call transmit when idle
//			return;
//	}
//
//	if (len > 0 && len < TX_BUF_SIZE) {
//		HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_text_buf, len);
//	}
//}

void start_transmission(void) {

	tx_packet_index = 0;								 // reset index
	transmit_sensor_packet_ITX(&system[tx_packet_index]); // transmit first packet
														 // SensorData data;
	// // __atomic_load(&[tx_packet_index], &data, __ATOMIC_SEQ_CST);
	// transmit_sensor_packet_IT(&data);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
#if PALM_IMU_ENABLE
	fingertip_runtime_on_uart_tx_complete(&g_fingertip_runtime, huart);
#endif

	if (huart->Instance == USART2) {
		switch (slave_tx_mode) {
			case TX_ACK_MODE:
				if (main_loop_state == READING_MODE) {
					slave_tx_mode = TX_READING_MODE;
				} else {
					slave_tx_mode = TX_IDLE;
				}
				break;
			case TX_CMD_MODE:
				slave_tx_mode = TX_IDLE;
				break;

			case TX_READING_MODE:
				slave_tx_mode = TX_READING_MODE;

				uint8_t current_count = __atomic_load_n(&system_count, __ATOMIC_SEQ_CST);

				// 移动到下一节车厢
				tx_packet_index++;

				if (tx_packet_index < current_count) {
					// 级联发送
					transmit_sensor_packet_ITX(&system[tx_packet_index]);
				} else {
					// 发送完毕，刹车
					tx_packet_index = 0;
				}
				break;

			case TX_IDLE:
				return;
		}
	}

	if (huart->Instance == USART1) {
		// do nothing
	}
}


void __UART_TxISR_8BIT(UART_HandleTypeDef* huart) {
	/* Disable the UART Transmit Data Register Empty Interrupt */
	ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
}

void tx_keep_alive(void) {
    static uint32_t last_check = 0;
    uint32_t current_time = HAL_GetTick();

    if (slave_tx_mode != TX_READING_MODE) {
        return;
    }

    if (current_time - last_check >= tx_ctrl.actual_interval_ms) {
        last_check = current_time;

        if (should_send_now()) {
            // 🛑 核心防御：只有当“火车完全停靠（index==0）”并且“串口完全空闲”时，才允许发新车！
            // 如果 tx_packet_index != 0，说明后面的子板 1 或 2 还没发完，绝对不能打断！
            if (tx_packet_index == 0 && huart2.gState == HAL_UART_STATE_READY) {
                transmit_sensor_packet_ITX(&system[0]);
            }
        }
    }
}


// 获取下一个发送时间
uint8_t should_send_now(void) {

    uint32_t current_time = HAL_GetTick();

    // 检查是否到达发送时间
    if (current_time - tx_ctrl.last_send_time >= tx_ctrl.actual_interval_ms) {
        tx_ctrl.last_send_time = current_time;
        return 1;
    }

    return 0;
}




