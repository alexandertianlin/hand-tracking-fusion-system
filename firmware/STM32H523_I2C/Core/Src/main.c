/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "icache.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "flash_eeprom.h"
#include "app/fingertip_runtime.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t STMFLASH_LastPage_BUF[MCU_PAGE_SIZE/16];//2048x4/16=1024/2 64bit, it mean that 1024/2 double 64bit calibration parameters in a half page
uint32_t STMFLASH_LastPage_BUF_TEMP[MCU_PAGE_SIZE/16];//backup buffer when erasing page
STM_HOST_DATA Code_data_A;
STM_HOST_DATA Code_data_B;
uint32_t A_BUF[2];
uint32_t B_BUF[2];
uint32_t STMFLASH_BUF[4];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
fingertip_runtime_t g_fingertip_runtime;
volatile HAL_StatusTypeDef g_imu_runtime_init_status = HAL_ERROR;
extern uint8_t calibration_uart_rx_buffer[3+152*ARRAY_PS];//calibration buffer
extern volatile uint8_t Cali_flag;//flag for calibration data
uint32_t temp_buffer[2];//calibration temp buffer
extern uint8_t uart_rx_buffer;
extern uint8_t master_rx_buffer;
extern SensorData system[MAX_BOARDS];
extern volatile uint8_t g_data_read_flag;
extern volatile uint8_t g_coil_read_flag;	//flag for mlx90393 timer3 225hz for coil force freq
extern send_controller_t tx_ctrl;
extern uint8_t system_count;//due to N sensors in serial
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void read_mlx90393_burst(void);
void tx_keep_alive(void);
void start_transmission(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CRC Calculation include the start byte!
static uint8_t calc_crc1(const uint8_t* data, uint16_t len) {
	uint8_t crc = 0;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
	}
	return crc;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ICACHE_Init();
  MX_I2C1_Init();
  if (fingertip_runtime_init(&g_fingertip_runtime, &hi2c1, &huart2) != HAL_OK)
    {
        // 初始化失败处理，防止系统运行在错误状态
        Error_Handler();
    }

  /* USER CODE BEGIN 2 */
	// Initialize mlx90393
  if (ARRAY_PS ==1)
  {
		// EX
		uint8_t cmd = 0x80;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);

		// RT
		cmd = 0xF0;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);


		uint16_t reg00_val = (0 << 8) | 0x1C;
		uint8_t buff0[4] = {0x60, (uint8_t)(reg00_val >> 8), (uint8_t)(reg00_val & 0xFF), 0x00 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff0, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff0, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff0, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff0, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		uint16_t reg02_val = (0x02 << 8) | 0xA8;
		uint8_t buff2[4] = {0x60, (uint8_t)(reg02_val >> 8), (uint8_t)(reg02_val & 0xFF), 0x02 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff2, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff2, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff2, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff2, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		// start burst
#if MLX90393_TEMP_READING
		uint8_t Burst_CMD = 0x1F;//Enable temperature reading
#else
		uint8_t Burst_CMD = 0x1E;
#endif
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &Burst_CMD, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &Burst_CMD, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &Burst_CMD, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &Burst_CMD, 1, HAL_MAX_DELAY);

		HAL_Delay(10);
		//flash last page initial start to read flash data to calibration parameters K0...
			  FLASH_ReadData(CAL_START_ADDRESS, temp_buffer, 2);//read K0_0
				memcpy(&K0_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+8, temp_buffer, 2);//read K1_0
				memcpy(&K1_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+16, temp_buffer, 2);//read K2_0
				memcpy(&K2_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+24, temp_buffer, 2);//read K3_0
				memcpy(&K3_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+32, temp_buffer, 2);//read K4_0
				memcpy(&K4_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+40, temp_buffer, 2);//read K5_0
				memcpy(&K5_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+48, temp_buffer, 2);//read K6_0
				memcpy(&K6_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+56, temp_buffer, 2);//read K7_0
				memcpy(&K7_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+64, temp_buffer, 2);//read K8_0
				memcpy(&K8_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+72, temp_buffer, 2);//read K9_0
				memcpy(&K9_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+80, temp_buffer, 2);//read a_x_0
				memcpy(&a_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+88, temp_buffer, 2);//read b_x_0
				memcpy(&b_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+96, temp_buffer, 2);//read c_x_0
				memcpy(&c_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+104, temp_buffer, 2);//read a_y_0
				memcpy(&a_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+112, temp_buffer, 2);//read b_y_0
				memcpy(&b_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+120, temp_buffer, 2);//read c_y_0
				memcpy(&c_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+128, temp_buffer, 2);//read a_z_0
				memcpy(&a_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+136, temp_buffer, 2);//read b_z_0
				memcpy(&b_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+144, temp_buffer, 2);//read c_z_0
				memcpy(&c_z_0, &temp_buffer, sizeof(double));

  }
  else if (ARRAY_PS ==2)
  {
		// EX
		uint8_t cmd = 0x80;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);

		// RT
		cmd = 0xF0;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);


		uint16_t reg00_val = (0 << 8) | 0x1C;
		uint8_t buff0[4] = {0x60, (uint8_t)(reg00_val >> 8), (uint8_t)(reg00_val & 0xFF), 0x00 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff0, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff0, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff0, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff0, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		uint16_t reg02_val = (0x02 << 8) | 0xA8;
		uint8_t buff2[4] = {0x60, (uint8_t)(reg02_val >> 8), (uint8_t)(reg02_val & 0xFF), 0x02 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff2, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff2, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff2, 4, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff2, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		// start burst
		uint8_t Burst_CMD = 0x1E;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &Burst_CMD, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &Burst_CMD, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &Burst_CMD, 1, HAL_MAX_DELAY);
	//	HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &Burst_CMD, 1, HAL_MAX_DELAY);

		HAL_Delay(10);
		//flash last page initial start to read flash data to calibration parameters K0...
			  FLASH_ReadData(CAL_START_ADDRESS, temp_buffer, 2);//read K0_0
				memcpy(&K0_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+8, temp_buffer, 2);//read K1_0
				memcpy(&K1_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+16, temp_buffer, 2);//read K2_0
				memcpy(&K2_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+24, temp_buffer, 2);//read K3_0
				memcpy(&K3_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+32, temp_buffer, 2);//read K4_0
				memcpy(&K4_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+40, temp_buffer, 2);//read K5_0
				memcpy(&K5_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+48, temp_buffer, 2);//read K6_0
				memcpy(&K6_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+56, temp_buffer, 2);//read K7_0
				memcpy(&K7_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+64, temp_buffer, 2);//read K8_0
				memcpy(&K8_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+72, temp_buffer, 2);//read K9_0
				memcpy(&K9_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+80, temp_buffer, 2);//read a_x_0
				memcpy(&a_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+88, temp_buffer, 2);//read b_x_0
				memcpy(&b_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+96, temp_buffer, 2);//read c_x_0
				memcpy(&c_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+104, temp_buffer, 2);//read a_y_0
				memcpy(&a_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+112, temp_buffer, 2);//read b_y_0
				memcpy(&b_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+120, temp_buffer, 2);//read c_y_0
				memcpy(&c_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+128, temp_buffer, 2);//read a_z_0
				memcpy(&a_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+136, temp_buffer, 2);//read b_z_0
				memcpy(&b_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+144, temp_buffer, 2);//read c_z_0
				memcpy(&c_z_0, &temp_buffer, sizeof(double));

				  FLASH_ReadData(CAL_START_ADDRESS+144+8, temp_buffer, 2);//read K0_1
					memcpy(&K0_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+16, temp_buffer, 2);//read K1_1
					memcpy(&K1_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+24, temp_buffer, 2);//read K2_1
					memcpy(&K2_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+32, temp_buffer, 2);//read K3_1
					memcpy(&K3_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+40, temp_buffer, 2);//read K4_1
					memcpy(&K4_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+48, temp_buffer, 2);//read K5_1
					memcpy(&K5_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+56, temp_buffer, 2);//read K6_1
					memcpy(&K6_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+64, temp_buffer, 2);//read K7_1
					memcpy(&K7_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+72, temp_buffer, 2);//read K8_1
					memcpy(&K8_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+80, temp_buffer, 2);//read K9_1
					memcpy(&K9_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+88, temp_buffer, 2);//read a_x_1
					memcpy(&a_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+96, temp_buffer, 2);//read b_x_1
					memcpy(&b_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+104, temp_buffer, 2);//read c_x_1
					memcpy(&c_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+112, temp_buffer, 2);//read a_y_1
					memcpy(&a_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+120, temp_buffer, 2);//read b_y_1
					memcpy(&b_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+128, temp_buffer, 2);//read c_y_1
					memcpy(&c_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+136, temp_buffer, 2);//read a_z_1
					memcpy(&a_z_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+144, temp_buffer, 2);//read b_z_1
					memcpy(&b_z_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+152, temp_buffer, 2);//read c_z_1
					memcpy(&c_z_1, &temp_buffer, sizeof(double));
  }
  else if (ARRAY_PS ==4)
  {
		// EX
		uint8_t cmd = 0x80;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);

		// RT
		cmd = 0xF0;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &cmd, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &cmd, 1, HAL_MAX_DELAY);
		HAL_Delay(10);


		uint16_t reg00_val = (0 << 8) | 0x1C;
		uint8_t buff0[4] = {0x60, (uint8_t)(reg00_val >> 8), (uint8_t)(reg00_val & 0xFF), 0x00 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff0, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff0, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff0, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff0, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		uint16_t reg02_val = (0x02 << 8) | 0xA8;
		uint8_t buff2[4] = {0x60, (uint8_t)(reg02_val >> 8), (uint8_t)(reg02_val & 0xFF), 0x02 << 2};
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, buff2, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, buff2, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, buff2, 4, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, buff2, 4, HAL_MAX_DELAY);
		HAL_Delay(10);

		// start burst
		uint8_t Burst_CMD = 0x1E;
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_1, &Burst_CMD, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_2, &Burst_CMD, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_3, &Burst_CMD, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDRESS_4, &Burst_CMD, 1, HAL_MAX_DELAY);

		HAL_Delay(10);
		//flash last page initial start to read flash data to calibration parameters K0...
			  FLASH_ReadData(CAL_START_ADDRESS, temp_buffer, 2);//read K0_0
				memcpy(&K0_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+8, temp_buffer, 2);//read K1_0
				memcpy(&K1_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+16, temp_buffer, 2);//read K2_0
				memcpy(&K2_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+24, temp_buffer, 2);//read K3_0
				memcpy(&K3_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+32, temp_buffer, 2);//read K4_0
				memcpy(&K4_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+40, temp_buffer, 2);//read K5_0
				memcpy(&K5_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+48, temp_buffer, 2);//read K6_0
				memcpy(&K6_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+56, temp_buffer, 2);//read K7_0
				memcpy(&K7_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+64, temp_buffer, 2);//read K8_0
				memcpy(&K8_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+72, temp_buffer, 2);//read K9_0
				memcpy(&K9_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+80, temp_buffer, 2);//read a_x_0
				memcpy(&a_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+88, temp_buffer, 2);//read b_x_0
				memcpy(&b_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+96, temp_buffer, 2);//read c_x_0
				memcpy(&c_x_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+104, temp_buffer, 2);//read a_y_0
				memcpy(&a_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+112, temp_buffer, 2);//read b_y_0
				memcpy(&b_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+120, temp_buffer, 2);//read c_y_0
				memcpy(&c_y_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+128, temp_buffer, 2);//read a_z_0
				memcpy(&a_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+136, temp_buffer, 2);//read b_z_0
				memcpy(&b_z_0, &temp_buffer, sizeof(double));
			  FLASH_ReadData(CAL_START_ADDRESS+144, temp_buffer, 2);//read c_z_0
				memcpy(&c_z_0, &temp_buffer, sizeof(double));

				  FLASH_ReadData(CAL_START_ADDRESS+144+8, temp_buffer, 2);//read K0_1
					memcpy(&K0_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+16, temp_buffer, 2);//read K1_1
					memcpy(&K1_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+24, temp_buffer, 2);//read K2_1
					memcpy(&K2_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+32, temp_buffer, 2);//read K3_1
					memcpy(&K3_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+40, temp_buffer, 2);//read K4_1
					memcpy(&K4_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+48, temp_buffer, 2);//read K5_1
					memcpy(&K5_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+56, temp_buffer, 2);//read K6_1
					memcpy(&K6_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+64, temp_buffer, 2);//read K7_1
					memcpy(&K7_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+72, temp_buffer, 2);//read K8_1
					memcpy(&K8_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+80, temp_buffer, 2);//read K9_1
					memcpy(&K9_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+88, temp_buffer, 2);//read a_x_1
					memcpy(&a_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+96, temp_buffer, 2);//read b_x_1
					memcpy(&b_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+104, temp_buffer, 2);//read c_x_1
					memcpy(&c_x_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+112, temp_buffer, 2);//read a_y_1
					memcpy(&a_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+120, temp_buffer, 2);//read b_y_1
					memcpy(&b_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+128, temp_buffer, 2);//read c_y_1
					memcpy(&c_y_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+136, temp_buffer, 2);//read a_z_1
					memcpy(&a_z_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+144, temp_buffer, 2);//read b_z_1
					memcpy(&b_z_1, &temp_buffer, sizeof(double));
				  FLASH_ReadData(CAL_START_ADDRESS+144+152, temp_buffer, 2);//read c_z_1
					memcpy(&c_z_1, &temp_buffer, sizeof(double));

					  FLASH_ReadData(CAL_START_ADDRESS+144+152+8, temp_buffer, 2);//read K0_2
						memcpy(&K0_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+16, temp_buffer, 2);//read K1_2
						memcpy(&K1_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+24, temp_buffer, 2);//read K2_2
						memcpy(&K2_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+32, temp_buffer, 2);//read K3_2
						memcpy(&K3_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+40, temp_buffer, 2);//read K4_2
						memcpy(&K4_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+48, temp_buffer, 2);//read K5_2
						memcpy(&K5_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+56, temp_buffer, 2);//read K6_2
						memcpy(&K6_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+64, temp_buffer, 2);//read K7_2
						memcpy(&K7_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+72, temp_buffer, 2);//read K8_2
						memcpy(&K8_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+80, temp_buffer, 2);//read K9_2
						memcpy(&K9_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+88, temp_buffer, 2);//read a_x_2
						memcpy(&a_x_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+96, temp_buffer, 2);//read b_x_2
						memcpy(&b_x_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+104, temp_buffer, 2);//read c_x_2
						memcpy(&c_x_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+112, temp_buffer, 2);//read a_y_2
						memcpy(&a_y_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+120, temp_buffer, 2);//read b_y_2
						memcpy(&b_y_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+128, temp_buffer, 2);//read c_y_2
						memcpy(&c_y_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+136, temp_buffer, 2);//read a_z_2
						memcpy(&a_z_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+144, temp_buffer, 2);//read b_z_2
						memcpy(&b_z_1, &temp_buffer, sizeof(double));
					  FLASH_ReadData(CAL_START_ADDRESS+144+152+152, temp_buffer, 2);//read c_z_2
						memcpy(&c_z_1, &temp_buffer, sizeof(double));

						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+8, temp_buffer, 2);//read K0_3
							memcpy(&K0_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+16, temp_buffer, 2);//read K1_3
							memcpy(&K1_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+24, temp_buffer, 2);//read K2_3
							memcpy(&K2_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+32, temp_buffer, 2);//read K3_3
							memcpy(&K3_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+40, temp_buffer, 2);//read K4_3
							memcpy(&K4_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+48, temp_buffer, 2);//read K5_3
							memcpy(&K5_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+56, temp_buffer, 2);//read K6_3
							memcpy(&K6_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+64, temp_buffer, 2);//read K7_3
							memcpy(&K7_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+72, temp_buffer, 2);//read K8_3
							memcpy(&K8_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+80, temp_buffer, 2);//read K9_3
							memcpy(&K9_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+88, temp_buffer, 2);//read a_x_3
							memcpy(&a_x_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+96, temp_buffer, 2);//read b_x_3
							memcpy(&b_x_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+104, temp_buffer, 2);//read c_x_3
							memcpy(&c_x_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+112, temp_buffer, 2);//read a_y_3
							memcpy(&a_y_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+120, temp_buffer, 2);//read b_y_3
							memcpy(&b_y_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+128, temp_buffer, 2);//read c_y_3
							memcpy(&c_y_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+136, temp_buffer, 2);//read a_z_3
							memcpy(&a_z_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+144, temp_buffer, 2);//read b_z_3
							memcpy(&b_z_1, &temp_buffer, sizeof(double));
						  FLASH_ReadData(CAL_START_ADDRESS+144+152+152+152, temp_buffer, 2);//read c_z_3
							memcpy(&c_z_1, &temp_buffer, sizeof(double));
  }
  else// Max. I2C 90393 = 4
  {
	  HAL_Delay(1);
  }
//read sys config parameters start
												FLASH_ReadData(SYS_START_ADDRESS, (uint32_t*)temp_buffer, 2);//read A code sys config@the first address ex:0XFFFF FFFF 0XFFFF FFFF
												memcpy(&A_BUF, temp_buffer, sizeof(double));
												Code_data_A.MCU_UPGRADE_FLAG = temp_buffer[0];
												Code_data_A.BOOTLOADER_VERSION = temp_buffer[0] >> 8;
												Code_data_A.SOFTWARE_VERSION1 = temp_buffer[0] >> 16;
												Code_data_A.SOFTWARE_VERSION2 = temp_buffer[0] >> 24;

												Code_data_A.CODE_ZONE = temp_buffer[1];
												Code_data_A.HOST_RESET = temp_buffer[1] >> 8;
												Code_data_A.HARDWARE_VERSION1 = temp_buffer[1] >> 16;
												Code_data_A.HARDWARE_VERSION2 = temp_buffer[1] >> 24;

												FLASH_ReadData(SYS_START_ADDRESS+8, (uint32_t*)temp_buffer, 2);//read B code sys config@the second address ex:0XFFFF FFFF 0XFFFF FFFF
												memcpy(&B_BUF, temp_buffer, sizeof(double));
												Code_data_B.MCU_UPGRADE_FLAG = temp_buffer[0];
												Code_data_B.BOOTLOADER_VERSION = temp_buffer[0] >> 8;
												Code_data_B.SOFTWARE_VERSION1 = temp_buffer[0] >> 16;
												Code_data_B.SOFTWARE_VERSION2 = temp_buffer[0] >> 24;

												Code_data_B.CODE_ZONE = temp_buffer[1];
												Code_data_B.HOST_RESET = temp_buffer[1] >> 8;
												Code_data_B.HARDWARE_VERSION1 = temp_buffer[1] >> 16;
												Code_data_B.HARDWARE_VERSION2 = temp_buffer[1] >> 24;
//read sys config parameters end
	// Start timers
	HAL_TIM_Base_Start_IT(&htim2); // 450Hz for sensor reading
	HAL_TIM_Base_Start_IT(&htim3); // 250Hz for FET1 control coil to magnetic force
	// Start UART reception
	// Each time receive 1 board data, use HAL_UART_RxCpltCallback to process the data
	HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1);
	HAL_UART_Receive_IT(&huart2, &master_rx_buffer, 1);//lpuart1 == uart2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	start_transmission();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if PALM_IMU_ENABLE
	  fingertip_runtime_process(&g_fingertip_runtime);//IMU drivers
#endif
	  //flash parameters start

	  		if (Cali_flag == 1)
	  		{//cali_flag start
	  			  uint8_t feedback_flash_success[4] = {0xAA, 0x55, 0x11, 0x00};//flash success
	  			  uint8_t feedback_flash_fail[4] = {0xBB, 0x66, 0x22, 0x11};//flash fail
	  			Cali_flag = 0;
	  			uint8_t computed_crc = calc_crc1(calibration_uart_rx_buffer, sizeof(calibration_uart_rx_buffer)-2);
	  		    if (computed_crc == calibration_uart_rx_buffer[3+152*ARRAY_PS-2])
	  		    {//crc start

	  		    		for (uint8_t i = 0; i < 19*ARRAY_PS; i++)//19*ARRAY_PS pcs cali parameters
	  		    		{
	  		    			memcpy(&STMFLASH_LastPage_BUF[i], &calibration_uart_rx_buffer[8*i+1], sizeof(double));
	  		    		}

  		    			if (STMFLASH_LastPage_BUF[0] == 0xFFFF3412)//this is not cali, but reset to download flag
  		    			{ //reset to download start
  		    				uint8_t feedback_reset[4] = {0xCC, 0x66, 0x66, 0xCC};//reset to download flag success
  		    				//before erasing flash, read existing cali parameters
  		  			    	Code_data_A.HOST_RESET = 0x01;//request reset and download new code
  		  		    		Code_data_B.HOST_RESET = 0x01;//request reset and download new code
  		    				HAL_FLASH_Unlock();
  		    				FLASH_Erase_Page(1, 31, 2);// bank2 lastpage31 erasing 1page
		    					FLASH_WriteData(CAL_START_ADDRESS, STMFLASH_LastPage_BUF_TEMP, MCU_PAGE_SIZE/16);//
		    					//write sys config parameters
		    						    		    				memcpy(&STMFLASH_BUF, &Code_data_A, sizeof(double));
		    						    		    				FLASH_WriteData(SYS_START_ADDRESS, STMFLASH_BUF, 2);
		    						    		    				memcpy(&STMFLASH_BUF, &Code_data_B, sizeof(double));
		    						    		    				FLASH_WriteData(SYS_START_ADDRESS+8, STMFLASH_BUF, 2);
		    						    		    				HAL_FLASH_Lock();
		    						    		  			    	HAL_UART_Transmit_IT(&huart2, feedback_reset, sizeof(feedback_reset));//output flash reset

  		  		    			//software reset start
  		  			    	HAL_Delay(10);
  		  			    		__disable_irq();//close all IT
  		  			    		__NVIC_SystemReset();//software reset
  		  			    		while(1);
  		  			    		//software reset end
  		  			       //reset to download end

  		    			}
  		    			else
  		    			{//cali start
	  	  		    		HAL_FLASH_Unlock();
	  	  		    		FLASH_Erase_Page(1, 31, 2);// bank2 lastpage31 erasing 1page
	  	  		    		FLASH_WriteData(CAL_START_ADDRESS, STMFLASH_LastPage_BUF, MCU_PAGE_SIZE/16);
	  	  		    		HAL_FLASH_Lock();

	  	  		    		FLASH_ReadData(CAL_START_ADDRESS, temp_buffer, 2);//only verify K0_0
	  	  		    		memcpy(&K0_0, &temp_buffer, sizeof(double));
	  	  		    		uint8_t verify_flash;
	  	  		    		verify_flash = memcmp(&K0_0, &STMFLASH_LastPage_BUF[0], sizeof(double));
	  	  		    		if (verify_flash == 0)
	  	  		    			{
	  	  			    	HAL_UART_Transmit_IT(&huart2, feedback_flash_success, sizeof(feedback_flash_success));//output special feedback 4 bytes
	  	  			        HAL_Delay(50);
	  	  		    		HAL_UART_Transmit_IT(&huart2, calibration_uart_rx_buffer, sizeof(calibration_uart_rx_buffer));//output back cali_data
	  	  		    			}
	  	  		    		else
	  	  		    			{
	  	  			    	HAL_UART_Transmit_IT(&huart2, feedback_flash_fail, sizeof(feedback_flash_fail));//output flash fail special feedback 4 bytes
	  	  		    			}
	    			}//cali end
	  		    }//crc end
	  		}//cali_flag end
//flash parameters end
	  		//force coil magnet by 225hz
	  		//	  				if (__atomic_load_n(&g_coil_read_flag, __ATOMIC_SEQ_CST))
	  		//	  				{
	  		//	  					  __atomic_store_n(&g_coil_read_flag, 0, __ATOMIC_SEQ_CST);
	  		//	  					  HAL_GPIO_TogglePin(FET1_GPIO_Port, FET1_Pin);// pwm 225hz
	  		//	  					  read_mlx90393_burst();
	  		//	  				}//force coil magnet by 225hz end
	  		//magnet by 450hz
	  			  	  			if (__atomic_load_n(&g_data_read_flag, __ATOMIC_SEQ_CST))
	  			  	  			{
	  			  	  				__atomic_store_n(&g_data_read_flag, 0, __ATOMIC_SEQ_CST);
	  			  	  			read_mlx90393_burst();
	  			  	  			}//magnet by 450hz end
	  			  			//process_sensor_data();
	  			  		  	if(tx_ctrl.actual_interval_ms !=0)
	  			  		  	{     tx_keep_alive();
	  			  		  	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
