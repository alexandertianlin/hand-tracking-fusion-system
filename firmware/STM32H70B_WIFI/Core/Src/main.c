/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @function	6UART_in and 1USB_out and 1uart1_out and uart1 drives WIFI AT cmd
 * @author	TOMSUN
 * @Agilereach	copyright
 * @date	20260410
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app/external_node_rx.h"
#include "app/palm_runtime.h"
#include "usb_process.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  WIFI_INIT_IDLE = 0,
  WIFI_INIT_VERSION,
  WIFI_INIT_MODE,
  WIFI_INIT_SCAN,
  WIFI_INIT_JOIN,
  WIFI_INIT_STATE,
  WIFI_INIT_IP,
  WIFI_INIT_TCP,
  WIFI_INIT_SENDLEN,
  WIFI_INIT_SENDDATA,
  WIFI_INIT_DONE
} wifi_init_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;//data out
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;//wifi cmd
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t usb_receive_buffer[128] = {0};
uint8_t usb_receive_buffer_index = 0;
uint8_t g_usb_receive_flag = 0;

static palm_runtime_t g_palm_runtime;
static wifi_init_state_t g_wifi_init_state = WIFI_INIT_IDLE;
static uint32_t g_wifi_init_next_tick = 0U;
static uint8_t g_wifi_init_started = 0U;
static external_node_frame_t g_external_node_pending_frame;
static uint8_t g_external_node_pending_valid = 0U;

 uint8_t g_LPUART1_data_out_flag = 0;
 uint8_t g_USB_data_out_flag = 1;
// uint8_t g_WIFI_data_out_flag = 1;
uint8_t g_B2_flag;
uint8_t g_B3_flag;
uint8_t g_B4_flag;
uint8_t g_B5_flag;
uint8_t g_B6_flag;
uint8_t g_B7_flag;

/* Request flag serviced from the main loop to arm or clear the palm-side
 * Zero All reference layer. Written by the USB command handler, consumed
 * here so that palm_runtime and external_node_rx are touched only from the
 * main-loop context. */
volatile uint8_t g_glove_zero_request = GLOVE_ZERO_REQUEST_NONE;

 uint8_t U7_rx[Protocol_len_64];  //uart7 64byte
 uint8_t U2_rx[Protocol_len];     //uart2 packet 16byte
 uint8_t U3_rx[Protocol_len];     //uart3 packet 16byte
 uint8_t U4_rx[Protocol_len];     //uart4 packet 16byte
 uint8_t U5_rx[Protocol_len];     //uart5 packet 16byte
 uint8_t U6_rx[Protocol_len];     //uart6 packet 16byte


 extern uint8_t uart3_rx_buffer_ready;
 extern uint8_t uart4_rx_buffer_ready;
 extern uint8_t uart5_rx_buffer_ready;
 extern uint8_t uart6_rx_buffer_ready;
 extern uint8_t uart2_rx_buffer_ready;
 extern uint8_t uart7_rx_buffer_ready;

 extern uint8_t Temp2_Header;
 extern uint8_t Temp3_Header;
 extern uint8_t Temp4_Header;
 extern uint8_t Temp5_Header;
 extern uint8_t Temp6_Header;
 extern uint8_t Temp7_Header;

 extern uint8_t lpuart1_rx_buffer;
 extern uint8_t uart1_rx_buffer;
 extern uint8_t uart3_rx_buffer;
 extern uint8_t uart4_rx_buffer;
 extern uint8_t uart5_rx_buffer;
 extern uint8_t uart6_rx_buffer;
 extern uint8_t uart2_rx_buffer;
 extern uint8_t uart7_rx_buffer;
extern USBD_HandleTypeDef hUsbDeviceHS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t calc_crc_1(uint8_t* data, uint8_t len);
void ESPC3MINI_Init(void);
void Send_AT_CMD(char *cmd);
static void ESPC3MINI_Init_Start(void);
static void ESPC3MINI_Init_Service(void);
static void External_Node_Service_USB(void);


static uint32_t glove_zero_startup_deadline_ms = 0;
static uint8_t glove_zero_startup_pending = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t calc_crc_1(uint8_t* data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}
void Send_AT_CMD(char *cmd)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
}

static void External_Node_Service_USB(void)
{
  if ((PALM_EXTERNAL_NODE_RX_ENABLE == 0U) || (g_USB_data_out_flag != 1U)) {
    return;
  }

  if (g_external_node_pending_valid == 0U) {
    if (external_node_rx_pop_frame(&g_external_node_pending_frame) == HAL_OK) {
      g_external_node_pending_valid = 1U;
    } else {
      return;
    }
  }

  if (hUsbDeviceHS.pClassData == NULL) {
    return;
  }

  if (CDC_Transmit_HS(g_external_node_pending_frame.frame,
                      PALM_PROTOCOL_FRAME_SIZE) == USBD_OK) {
    g_external_node_pending_valid = 0U;
  }
}

void ESPC3MINI_Init(void)
{
//	Send_AT_CMD("AT+RST\r\n");//reset
//	HAL_Delay(2000);
	Send_AT_CMD("AT+GMR\r\n");//version
	HAL_Delay(100);
	Send_AT_CMD("AT+CWMODE=1\r\n");//station mode
	HAL_Delay(100);
	Send_AT_CMD("AT+CWLAP\r\n");//scan all wifi list
	HAL_Delay(1000);
	Send_AT_CMD("AT+CWJAP=\"413-2\",\"agilereach888\"\r\n");//connection to wifi
	HAL_Delay(1000);
	Send_AT_CMD("AT+CWSTATE?\r\n");//
	HAL_Delay(1000);
	Send_AT_CMD("AT+CIPSTA?\r\n");//
	HAL_Delay(1000);
	Send_AT_CMD("AT+CIPSTART=\"TCP\",\"192.168.1.113\",8080\r\n");//connect to tcp server
	HAL_Delay(2000);
	Send_AT_CMD("AT+CIPSEND=16\r\n");//send 16byte for initializing
	HAL_Delay(1000);
	Send_AT_CMD("0123456789ABCDEF\r\n");//send 16byte
	HAL_Delay(1000);
}

static void ESPC3MINI_Init_Start(void)
{
  g_wifi_init_started = 1U;
  g_wifi_init_state = WIFI_INIT_VERSION;
  g_wifi_init_next_tick = HAL_GetTick() + 200U;
}

static void ESPC3MINI_Init_Service(void)
{
  uint32_t now_ms;

  if ((g_wifi_init_started == 0U) || (g_wifi_init_state == WIFI_INIT_DONE)) {
    return;
  }

  now_ms = HAL_GetTick();
  if ((int32_t)(now_ms - g_wifi_init_next_tick) < 0) {
    return;
  }

  switch (g_wifi_init_state) {
    case WIFI_INIT_VERSION:
      Send_AT_CMD("AT+GMR\r\n");
      g_wifi_init_state = WIFI_INIT_MODE;
      g_wifi_init_next_tick = now_ms + 100U;
      break;

    case WIFI_INIT_MODE:
      Send_AT_CMD("AT+CWMODE=1\r\n");
      g_wifi_init_state = WIFI_INIT_SCAN;
      g_wifi_init_next_tick = now_ms + 100U;
      break;

    case WIFI_INIT_SCAN:
      Send_AT_CMD("AT+CWLAP\r\n");
      g_wifi_init_state = WIFI_INIT_JOIN;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_JOIN:
      Send_AT_CMD("AT+CWJAP=\"413-2\",\"agilereach888\"\r\n");
      g_wifi_init_state = WIFI_INIT_STATE;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_STATE:
      Send_AT_CMD("AT+CWSTATE?\r\n");
      g_wifi_init_state = WIFI_INIT_IP;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_IP:
      Send_AT_CMD("AT+CIPSTA?\r\n");
      g_wifi_init_state = WIFI_INIT_TCP;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_TCP:
      Send_AT_CMD("AT+CIPSTART=\"TCP\",\"192.168.1.113\",8080\r\n");
      g_wifi_init_state = WIFI_INIT_SENDLEN;
      g_wifi_init_next_tick = now_ms + 2000U;
      break;

    case WIFI_INIT_SENDLEN:
      Send_AT_CMD("AT+CIPSEND=16\r\n");
      g_wifi_init_state = WIFI_INIT_SENDDATA;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_SENDDATA:
      Send_AT_CMD("0123456789ABCDEF\r\n");
      g_wifi_init_state = WIFI_INIT_DONE;
      g_wifi_init_next_tick = now_ms + 1000U;
      break;

    case WIFI_INIT_IDLE:
    case WIFI_INIT_DONE:
    default:
      break;
  }
}
/* USER CODE END 0 */
/* USER CODE BEGIN 0 */

// 封装 1: Header 状态维护
void Update_Headers_Logic(void) {
    Temp2_Header = (__atomic_load_n(&g_B2_flag, __ATOMIC_RELAXED)) ? START_BYTE : ACK_BYTE;
    Temp3_Header = (__atomic_load_n(&g_B3_flag, __ATOMIC_RELAXED)) ? START_BYTE : ACK_BYTE;
    Temp4_Header = (__atomic_load_n(&g_B4_flag, __ATOMIC_RELAXED)) ? START_BYTE : ACK_BYTE;
    Temp5_Header = __atomic_load_n(&g_B5_flag, __ATOMIC_RELAXED) ? START_BYTE : ACK_BYTE;
    Temp6_Header = __atomic_load_n(&g_B6_flag, __ATOMIC_RELAXED) ? START_BYTE : ACK_BYTE;
    Temp7_Header = __atomic_load_n(&g_B7_flag, __ATOMIC_RELAXED) ? START_BYTE : ACK_BYTE;
}

// 封装 2: 串口转发 (把所有 LPUART1 的逻辑打包)
void Handle_UART_Forwarding(void) {
    if (g_LPUART1_data_out_flag != 1) return;

    if(uart2_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart2_rx_buffer_ready = 0;
    	    	    		volatile uint8_t crc_1 = 0;
    	    	    		for (uint8_t z = 1; z < 15; z++) {
    	    	    		crc_1 ^= U2_rx[z];
    	    	    		}
    	    	    		U2_rx[15] = crc_1;
    	    	    		if ((U2_rx[14] > 19)&&(U2_rx[14] < 30))//channel 2X
    	    	    		{

    	    	    		HAL_UART_Transmit(&hlpuart1, U2_rx, Protocol_len, HAL_MAX_DELAY);

    	    	    		}

    	    	    	}

    	    	    	if(uart3_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart3_rx_buffer_ready = 0;
    	    	    		volatile uint8_t crc_2 = 0;
    	    	    		for (uint8_t z = 1; z < 15; z++) {
    	    	    		crc_2 ^= U3_rx[z];
    	    	    		}
    	    	    		U3_rx[15] = crc_2;
    	    	    		if ((U3_rx[14] > 29)&&(U3_rx[14] < 40))//channel 3X
    	    	    		{
    		    	    		HAL_UART_Transmit(&hlpuart1, U3_rx, Protocol_len, HAL_MAX_DELAY);

    	    	    		}
    	    	    	}

    	    	    	if(uart4_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart4_rx_buffer_ready = 0;
    	    	    		volatile uint8_t crc_3 = 0;
    	    	    		for (uint8_t z = 1; z < 15; z++) {
    	    	    		crc_3 ^= U4_rx[z];
    	    	    		}
    	    	    		U4_rx[15] = crc_3;
    	    	    		if ((U4_rx[14] > 39)&&(U4_rx[14] < 50))//channel 4X
    	    	    		{
    		    	    		HAL_UART_Transmit(&hlpuart1, U4_rx, Protocol_len, HAL_MAX_DELAY);

    	    	    		}
    	    	    	}

    	    	    	if(uart5_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart5_rx_buffer_ready = 0;
    	    	    		volatile uint8_t crc_4 = 0;
    	    	    		for (uint8_t z = 1; z < 15; z++) {
    	    	    		crc_4 ^= U5_rx[z];
    	    	    		}
    	    	    		U5_rx[15] = crc_4;
    	    	    		if ((U5_rx[14] > 49)&&(U5_rx[14] < 60))//channel 5X
    	    	    		{
    		    	    		HAL_UART_Transmit(&hlpuart1, U5_rx, Protocol_len, HAL_MAX_DELAY);

    	    	    		}
    	    	    	}

    	    	    	if(uart6_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart6_rx_buffer_ready = 0;

    	    	    		volatile uint8_t crc_5 = 0;
    	    	    		for (uint8_t z = 1; z < 15; z++) {
    	    	    		crc_5 ^= U6_rx[z];
    	    	    		}
    	    	    		U6_rx[15] = crc_5;
    	    	    		if ((U6_rx[14] > 59)&&(U6_rx[14] < 70))//channel 6X
    	    	    		{
    		    	    		HAL_UART_Transmit(&hlpuart1, U6_rx, Protocol_len, HAL_MAX_DELAY);

    	    	    		}
    	    	    	}

    	    	    	if(uart7_rx_buffer_ready == 1)
    	    	    	{
    	    	    		uart7_rx_buffer_ready = 0;
    	    	    		volatile uint8_t crc_0 = 0;
    	    	    		for (uint8_t z = 1; z < 63; z++) {
    	    	    		crc_0 ^= U7_rx[z];
    	    	    		}
    	    	    		U7_rx[63] = crc_0;
    	    	    		if (U7_rx[62] == 70)//only device id = 70
    	    	    		{
    		    	    		HAL_UART_Transmit(&hlpuart1, U7_rx, Protocol_len_64, HAL_MAX_DELAY);
    	    	    		}

    	    	    	}
}

// 封装 3: USB 数据输出 (把原本 USB_data_out start 下的所有 if 块放进来)
void Handle_USB_Send_Logic(void) {
    if (g_USB_data_out_flag != 1) return;
    char Hex_Str[33];//16byte to str
    char Hex_Str_128[129];//64byte to str
        	    	    	if(uart2_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart2_rx_buffer_ready = 0;
        	    	    		volatile uint8_t crc_1 = 0;
        	    	    		for (uint8_t z = 1; z < 15; z++) {
        	    	    		crc_1 ^= U2_rx[z];
        	    	    		}
        	    	    		U2_rx[15] = crc_1;
        	    	    		if ((U2_rx[14] > 19)&&(U2_rx[14] < 30))//channel 2X
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U2_rx, Protocol_len);
            	    	    			for (uint8_t i = 0; i < 16; i++) {
           	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U2_rx[i]);
            	    	    			}
           	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
           	    	    	    		//HAL_Delay(10);
           	    	    	    		Send_AT_CMD(Hex_Str);
           	    	    	    		Send_AT_CMD("\r\n");
        	    	    		}

        	    	    	}

        	    	    	if(uart3_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart3_rx_buffer_ready = 0;
        	    	    		volatile uint8_t crc_2 = 0;
        	    	    		for (uint8_t z = 1; z < 15; z++) {
        	    	    		crc_2 ^= U3_rx[z];
        	    	    		}
        	    	    		U3_rx[15] = crc_2;
        	    	    		if ((U3_rx[14] > 29)&&(U3_rx[14] < 40))//channel 3X
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U3_rx, Protocol_len);
        	    	    				for (uint8_t i = 0; i < 16; i++) {
        	    	    					sprintf(&Hex_Str[i * 2], "%02X", U3_rx[i]);
        	    	    				}
        	    	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
        	    	    				//HAL_Delay(10);
        	    	    				Send_AT_CMD(Hex_Str);
        	    	    				Send_AT_CMD("\r\n");
        	    	    		}
        	    	    	}

        	    	    	if(uart4_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart4_rx_buffer_ready = 0;
        	    	    		volatile uint8_t crc_3 = 0;
        	    	    		for (uint8_t z = 1; z < 15; z++) {
        	    	    		crc_3 ^= U4_rx[z];
        	    	    		}
        	    	    		U4_rx[15] = crc_3;
        	    	    		if ((U4_rx[14] > 39)&&(U4_rx[14] < 50))//channel 4X
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U4_rx, Protocol_len);
        	    				for (uint8_t i = 0; i < 16; i++) {
        	    					sprintf(&Hex_Str[i * 2], "%02X", U4_rx[i]);
        	    				}
        	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
        	    				//HAL_Delay(10);
        	    				Send_AT_CMD(Hex_Str);
        	    				Send_AT_CMD("\r\n");
        	    	    		}
        	    	    	}

        	    	    	if(uart5_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart5_rx_buffer_ready = 0;
        	    	    		volatile uint8_t crc_4 = 0;
        	    	    		for (uint8_t z = 1; z < 15; z++) {
        	    	    		crc_4 ^= U5_rx[z];
        	    	    		}
        	    	    		U5_rx[15] = crc_4;
        	    	    		if ((U5_rx[14] > 49)&&(U5_rx[14] < 60))//channel 5X
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U5_rx, Protocol_len);
        	    				for (uint8_t i = 0; i < 16; i++) {
        	    					sprintf(&Hex_Str[i * 2], "%02X", U5_rx[i]);
        	    				}
        	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
        	    				//HAL_Delay(10);
        	    				Send_AT_CMD(Hex_Str);
        	    				Send_AT_CMD("\r\n");
        	    	    		}
        	    	    	}

        	    	    	if(uart6_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart6_rx_buffer_ready = 0;

        	    	    		volatile uint8_t crc_5 = 0;
        	    	    		for (uint8_t z = 1; z < 15; z++) {
        	    	    		crc_5 ^= U6_rx[z];
        	    	    		}
        	    	    		U6_rx[15] = crc_5;
        	    	    		if ((U6_rx[14] > 59)&&(U6_rx[14] < 70))//channel 6X
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U6_rx, Protocol_len);
        	    				for (uint8_t i = 0; i < 16; i++) {
        	    					sprintf(&Hex_Str[i * 2], "%02X", U6_rx[i]);
        	    				}
        	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
        	    				//HAL_Delay(10);
        	    				Send_AT_CMD(Hex_Str);
        	    				Send_AT_CMD("\r\n");
        	    	    		}
        	    	    	}

        	    	    	if(uart7_rx_buffer_ready == 1)
        	    	    	{
        	    	    		uart7_rx_buffer_ready = 0;
        	    	    		volatile uint8_t crc_0 = 0;
        	    	    		for (uint8_t z = 1; z < 63; z++) {
        	    	    		crc_0 ^= U7_rx[z];
        	    	    		}
        	    	    		U7_rx[63] = crc_0;
        	    	    		if (U7_rx[62] == 70)//only device id = 70
        	    	    		{
        	    	    		CDC_Transmit_HS((uint8_t*)U7_rx, Protocol_len_64);
        	    				for (uint8_t i = 0; i < 64; i++) {
        	    					sprintf(&Hex_Str[i * 2], "%02X", U7_rx[i]);
        	    				}
        	    				Send_AT_CMD("AT+CIPSEND=128\r\n");//send 128byte
        	    				//HAL_Delay(10);
        	    				Send_AT_CMD(Hex_Str_128);
        	    				Send_AT_CMD("\r\n");
        	    	    		}

        	    	    	}
}

// 封装 4: 零点服务
void Service_System_Requests(void) {
    // 1. 处理零点请求
    uint8_t zero_req = __atomic_load_n(&g_glove_zero_request, __ATOMIC_RELAXED);
    if (zero_req != GLOVE_ZERO_REQUEST_NONE) {
        if (zero_req == GLOVE_ZERO_REQUEST_ZERO) { palm_runtime_request_zero(&g_palm_runtime); external_node_rx_request_zero_all(); }
        else if (zero_req == GLOVE_ZERO_REQUEST_CLEAR) { palm_runtime_clear_zero(&g_palm_runtime); external_node_rx_clear_zero_all(); }
        __atomic_store_n(&g_glove_zero_request, GLOVE_ZERO_REQUEST_NONE, __ATOMIC_RELAXED);
    }
    // 2. 处理延迟启动
    if (glove_zero_startup_pending != 0U) {
        if ((int32_t)(HAL_GetTick() - glove_zero_startup_deadline_ms) >= 0) {
            palm_runtime_request_zero(&g_palm_runtime);
            external_node_rx_request_zero_all();
            glove_zero_startup_pending = 0U;
        }
    }
}

/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */

/* --- 新增：LPUART1 非阻塞发件箱 --- */
#define TX_FIFO_SIZE 20
#define MAX_PACKET_LEN 64 // 兼容 UART7 的最大长度

typedef struct {
    uint8_t data[MAX_PACKET_LEN];
    uint8_t len;
} LPUART_TxPacket_t;

LPUART_TxPacket_t lpuart_tx_fifo[TX_FIFO_SIZE];
volatile uint8_t tx_fifo_head = 0;
volatile uint8_t tx_fifo_tail = 0;

void LPUART1_Send_NonBlocking(uint8_t *data, uint8_t len) {
    uint8_t next_head = (tx_fifo_head + 1) % TX_FIFO_SIZE;
    if (next_head != tx_fifo_tail) {
        memcpy(lpuart_tx_fifo[tx_fifo_head].data, data, len);
        lpuart_tx_fifo[tx_fifo_head].len = len;
        tx_fifo_head = next_head;
    }
    if (hlpuart1.gState == HAL_UART_STATE_READY && tx_fifo_tail != tx_fifo_head) {
        HAL_UART_Transmit_IT(&hlpuart1, lpuart_tx_fifo[tx_fifo_tail].data, lpuart_tx_fifo[tx_fifo_tail].len);
    }
}

// ... 你的其他原本就在 BEGIN 0 里的函数 ...
/* USER CODE END 0 */
void Process_UART_Packet(uint8_t *rx_buf, UART_HandleTypeDef *huart, uint8_t *rx_flag) {
    if (*rx_flag == 1) {
        *rx_flag = 0;

        // 1. 计算 CRC
        uint8_t len = (huart->Instance == UART7) ? Protocol_len_64 : Protocol_len;
        uint8_t crc = 0;
        for (uint8_t z = 1; z < (len - 1); z++) {
            crc ^= rx_buf[z];
        }
        rx_buf[len - 1] = crc;

        // 2. 过滤并“秒入队” (这里耗时几乎为 0，再也不会卡住主循环！)
        if (huart->Instance == USART2 && rx_buf[14] > 19 && rx_buf[14] < 30) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
        else if (huart->Instance == USART3 && rx_buf[14] > 29 && rx_buf[14] < 40) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
        else if (huart->Instance == UART4 && rx_buf[14] > 39 && rx_buf[14] < 50) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
        else if (huart->Instance == UART5 && rx_buf[14] > 49 && rx_buf[14] < 60) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
        else if (huart->Instance == USART6 && rx_buf[14] > 59 && rx_buf[14] < 70) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
        else if (huart->Instance == UART7 && rx_buf[62] == 70) {
            LPUART1_Send_NonBlocking(rx_buf, len);
        }
    }
}

/* 此时你的 Update_Headers_Logic 和 Handle_USB_Send_Logic 保持不变即可 */
/* USER CODE END 0 */

/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//	__HAL_RCC_HSI_ENABLE();	//RC internal freq
//	while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY));	//waiting for stable

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
#if PALM_GLOVE_ZERO_AT_STARTUP
	        /* 现在这里调用 HAL_GetTick() 是合法的，因为这是运行时的赋值 */
	        glove_zero_startup_deadline_ms = HAL_GetTick() + PALM_GLOVE_ZERO_STARTUP_DELAY_MS;
	        glove_zero_startup_pending = 1U;
#else
	        glove_zero_startup_deadline_ms = 0U;
	        glove_zero_startup_pending = 0U;
#endif
  uint8_t mlx_start_cmd = 0xB1;

  HAL_UART_Transmit(&huart2, &mlx_start_cmd, 1, 100);
  HAL_UART_Transmit(&huart3, &mlx_start_cmd, 1, 100);
  HAL_UART_Transmit(&huart4, &mlx_start_cmd, 1, 100);
  HAL_UART_Transmit(&huart5, &mlx_start_cmd, 1, 100);
  HAL_UART_Transmit(&huart6, &mlx_start_cmd, 1, 100);
  HAL_UART_Transmit(&huart7, &mlx_start_cmd, 1, 100);

  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
			Temp7_Header = ACK_BYTE;//0xB5
	    	Temp2_Header = ACK_BYTE;//0xB5
	    	Temp3_Header = ACK_BYTE;//0xB5
	    	Temp4_Header = ACK_BYTE;//0xB5
	    	Temp5_Header = ACK_BYTE;//0xB5
	    	Temp6_Header = ACK_BYTE;//0xB5
	    	__atomic_store_n(&g_B2_flag, 0, __ATOMIC_RELAXED);// uart2 off
	    	__atomic_store_n(&g_B3_flag, 0, __ATOMIC_RELAXED);// uart3 off
	    	__atomic_store_n(&g_B4_flag, 0, __ATOMIC_RELAXED);// uart4 off
	    	__atomic_store_n(&g_B5_flag, 0, __ATOMIC_RELAXED);// uart5 off
	    	__atomic_store_n(&g_B6_flag, 0, __ATOMIC_RELAXED);// uart6 off
	    	__atomic_store_n(&g_B7_flag, 0, __ATOMIC_RELAXED);// uart7 off
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);//red led
   	  HAL_Delay(100);
   	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);//green led
   	  HAL_Delay(100);
   	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);//red led
   	  HAL_Delay(100);
   	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);//green led
   	  HAL_Delay(100);
   	  uint32_t led_tick = HAL_GetTick();

// Start UART reception IT
// Each time receive 1 byte data, use HAL_UART_RxCpltCallback to process the data
  external_node_rx_init();
  HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart2, &uart2_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart5, &uart5_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&huart7, &uart7_rx_buffer, 1);//start to IT receive 1 byte to buffer
  HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_buffer, 1);//start RS485 to IT receive 1 byte to buffer
  if (palm_runtime_init(&g_palm_runtime, &hi2c3) != HAL_OK)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  }
  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

      palm_runtime_process(&g_palm_runtime);
      External_Node_Service_USB();

      // 心跳
      if (HAL_GetTick() - led_tick > 100) {
        led_tick = HAL_GetTick();
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }

      // 2. 顺序轮询，防止本地数据通讯级饥饿
      if(uart2_rx_buffer_ready == 1) Process_UART_Packet(U2_rx, &huart2, &uart2_rx_buffer_ready);
      if(uart3_rx_buffer_ready == 1) Process_UART_Packet(U3_rx, &huart3, &uart3_rx_buffer_ready);
      if(uart4_rx_buffer_ready == 1) Process_UART_Packet(U4_rx, &huart4, &uart4_rx_buffer_ready);
      if(uart5_rx_buffer_ready == 1) Process_UART_Packet(U5_rx, &huart5, &uart5_rx_buffer_ready);
      if(uart6_rx_buffer_ready == 1) Process_UART_Packet(U6_rx, &huart6, &uart6_rx_buffer_ready);
      if(uart7_rx_buffer_ready == 1) Process_UART_Packet(U7_rx, &huart7, &uart7_rx_buffer_ready);

      Handle_USB_Send_Logic();

      if (__atomic_load_n(&g_usb_receive_flag, __ATOMIC_RELAXED)) Process_USB_Receive_Buffer();

      Update_Headers_Logic();      // 更新 Header 标志位
      ESPC3MINI_Init_Service();    // 处理 WiFi 状态机
      Service_System_Requests();   // 处理零点请求

    /* USER CODE END 3 */

	    //end to usb flag triggered
//start to change byte header
//	    if (__atomic_load_n(&g_B2_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp2_Header = START_BYTE;//0xB4;
//	    }
//	    else
//	    {
//		    	Temp2_Header = ACK_BYTE;//0xB5
//	    }
//	    if (__atomic_load_n(&g_B3_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp3_Header = START_BYTE;//0xB4;
//	    }
//	    else
//	    {
//		    	Temp3_Header = ACK_BYTE;//0xB5
//	    }
//	    if (__atomic_load_n(&g_B4_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp4_Header = START_BYTE;//0xB4;
//
//	    }
//	    else
//	    {
//		    	Temp4_Header = ACK_BYTE;//0xB5
//	    }
//	    if (__atomic_load_n(&g_B5_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp5_Header = START_BYTE;//0xB4;
//	    }
//	    else
//	    {
//		    	Temp5_Header = ACK_BYTE;//0xB5
//	    }
//	    if (__atomic_load_n(&g_B6_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp6_Header = START_BYTE;//0xB4;
//	    }
//	    else
//	    {
//		    	Temp6_Header = ACK_BYTE;//0xB5
//	    }
//	    if (__atomic_load_n(&g_B7_flag, __ATOMIC_RELAXED))
//	    {
//		    	Temp7_Header = START_BYTE;//0xB4;
//	    }
//	    else
//	    {
//	    	Temp7_Header = ACK_BYTE;//0xB5
//	    }
////
////lpuart1 flag triggered
//	    	    if (g_LPUART1_data_out_flag == 1)
//	    	    {
//	    	    	if(uart2_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart2_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_1 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_1 ^= U2_rx[z];
//	    	    		}
//	    	    		U2_rx[15] = crc_1;
//	    	    		if ((U2_rx[14] > 19)&&(U2_rx[14] < 30))//channel 2X
//	    	    		{
//
//	    	    		HAL_UART_Transmit(&hlpuart1, U2_rx, Protocol_len, HAL_MAX_DELAY);
//
//	    	    		}
//
//	    	    	}
//
//	    	    	if(uart3_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart3_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_2 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_2 ^= U3_rx[z];
//	    	    		}
//	    	    		U3_rx[15] = crc_2;
//	    	    		if ((U3_rx[14] > 29)&&(U3_rx[14] < 40))//channel 3X
//	    	    		{
//		    	    		HAL_UART_Transmit(&hlpuart1, U3_rx, Protocol_len, HAL_MAX_DELAY);
//
//	    	    		}
//	    	    	}
//
//	    	    	if(uart4_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart4_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_3 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_3 ^= U4_rx[z];
//	    	    		}
//	    	    		U4_rx[15] = crc_3;
//	    	    		if ((U4_rx[14] > 39)&&(U4_rx[14] < 50))//channel 4X
//	    	    		{
//		    	    		HAL_UART_Transmit(&hlpuart1, U4_rx, Protocol_len, HAL_MAX_DELAY);
//
//	    	    		}
//	    	    	}
//
//	    	    	if(uart5_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart5_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_4 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_4 ^= U5_rx[z];
//	    	    		}
//	    	    		U5_rx[15] = crc_4;
//	    	    		if ((U5_rx[14] > 49)&&(U5_rx[14] < 60))//channel 5X
//	    	    		{
//		    	    		HAL_UART_Transmit(&hlpuart1, U5_rx, Protocol_len, HAL_MAX_DELAY);
//
//	    	    		}
//	    	    	}
//
//	    	    	if(uart6_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart6_rx_buffer_ready = 0;
//
//	    	    		volatile uint8_t crc_5 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_5 ^= U6_rx[z];
//	    	    		}
//	    	    		U6_rx[15] = crc_5;
//	    	    		if ((U6_rx[14] > 59)&&(U6_rx[14] < 70))//channel 6X
//	    	    		{
//		    	    		HAL_UART_Transmit(&hlpuart1, U6_rx, Protocol_len, HAL_MAX_DELAY);
//
//	    	    		}
//	    	    	}
//
//	    	    	if(uart7_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart7_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_0 = 0;
//	    	    		for (uint8_t z = 1; z < 63; z++) {
//	    	    		crc_0 ^= U7_rx[z];
//	    	    		}
//	    	    		U7_rx[63] = crc_0;
//	    	    		if (U7_rx[62] == 70)//only device id = 70
//	    	    		{
//		    	    		HAL_UART_Transmit(&hlpuart1, U7_rx, Protocol_len_64, HAL_MAX_DELAY);
//	    	    		}
//
//	    	    	}
//	    	    }
//lpuart1 end
//USB_data_out start
//	    	    if (g_USB_data_out_flag == 1)
//	       		{
//    	    	    	char Hex_Str[33];//16byte to str
//    	    	    	char Hex_Str_128[129];//64byte to str
//	    	    	if(uart2_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart2_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_1 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_1 ^= U2_rx[z];
//	    	    		}
//	    	    		U2_rx[15] = crc_1;
//	    	    		if ((U2_rx[14] > 19)&&(U2_rx[14] < 30))//channel 2X
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U2_rx, Protocol_len);
//    	    	    			for (uint8_t i = 0; i < 16; i++) {
//   	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U2_rx[i]);
//    	    	    			}
//   	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//   	    	    	    		//HAL_Delay(10);
//   	    	    	    		Send_AT_CMD(Hex_Str);
//   	    	    	    		Send_AT_CMD("\r\n");
//	    	    		}
//
//	    	    	}
//
//	    	    	if(uart3_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart3_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_2 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_2 ^= U3_rx[z];
//	    	    		}
//	    	    		U3_rx[15] = crc_2;
//	    	    		if ((U3_rx[14] > 29)&&(U3_rx[14] < 40))//channel 3X
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U3_rx, Protocol_len);
//	    	    				for (uint8_t i = 0; i < 16; i++) {
//	    	    					sprintf(&Hex_Str[i * 2], "%02X", U3_rx[i]);
//	    	    				}
//	    	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    				//HAL_Delay(10);
//	    	    				Send_AT_CMD(Hex_Str);
//	    	    				Send_AT_CMD("\r\n");
//	    	    		}
//	    	    	}
//
//	    	    	if(uart4_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart4_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_3 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_3 ^= U4_rx[z];
//	    	    		}
//	    	    		U4_rx[15] = crc_3;
//	    	    		if ((U4_rx[14] > 39)&&(U4_rx[14] < 50))//channel 4X
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U4_rx, Protocol_len);
//	    				for (uint8_t i = 0; i < 16; i++) {
//	    					sprintf(&Hex_Str[i * 2], "%02X", U4_rx[i]);
//	    				}
//	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    				//HAL_Delay(10);
//	    				Send_AT_CMD(Hex_Str);
//	    				Send_AT_CMD("\r\n");
//	    	    		}
//	    	    	}
//
//	    	    	if(uart5_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart5_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_4 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_4 ^= U5_rx[z];
//	    	    		}
//	    	    		U5_rx[15] = crc_4;
//	    	    		if ((U5_rx[14] > 49)&&(U5_rx[14] < 60))//channel 5X
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U5_rx, Protocol_len);
//	    				for (uint8_t i = 0; i < 16; i++) {
//	    					sprintf(&Hex_Str[i * 2], "%02X", U5_rx[i]);
//	    				}
//	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    				//HAL_Delay(10);
//	    				Send_AT_CMD(Hex_Str);
//	    				Send_AT_CMD("\r\n");
//	    	    		}
//	    	    	}
//
//	    	    	if(uart6_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart6_rx_buffer_ready = 0;
//
//	    	    		volatile uint8_t crc_5 = 0;
//	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    		crc_5 ^= U6_rx[z];
//	    	    		}
//	    	    		U6_rx[15] = crc_5;
//	    	    		if ((U6_rx[14] > 59)&&(U6_rx[14] < 70))//channel 6X
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U6_rx, Protocol_len);
//	    				for (uint8_t i = 0; i < 16; i++) {
//	    					sprintf(&Hex_Str[i * 2], "%02X", U6_rx[i]);
//	    				}
//	    				Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    				//HAL_Delay(10);
//	    				Send_AT_CMD(Hex_Str);
//	    				Send_AT_CMD("\r\n");
//	    	    		}
//	    	    	}
//
//	    	    	if(uart7_rx_buffer_ready == 1)
//	    	    	{
//	    	    		uart7_rx_buffer_ready = 0;
//	    	    		volatile uint8_t crc_0 = 0;
//	    	    		for (uint8_t z = 1; z < 63; z++) {
//	    	    		crc_0 ^= U7_rx[z];
//	    	    		}
//	    	    		U7_rx[63] = crc_0;
//	    	    		if (U7_rx[62] == 70)//only device id = 70
//	    	    		{
//	    	    		CDC_Transmit_HS((uint8_t*)U7_rx, Protocol_len_64);
//	    				for (uint8_t i = 0; i < 64; i++) {
//	    					sprintf(&Hex_Str[i * 2], "%02X", U7_rx[i]);
//	    				}
//	    				Send_AT_CMD("AT+CIPSEND=128\r\n");//send 128byte
//	    				//HAL_Delay(10);
//	    				Send_AT_CMD(Hex_Str_128);
//	    				Send_AT_CMD("\r\n");
//	    	    		}
//
//	    	    	}
//	    	    }
//USB_uart end
////WIFI_data_out start
//   	    	    if (g_WIFI_data_out_flag == 1)
//   	       		{
//	    	    	    	    	char Hex_Str[33];//16byte to str
//	    	    	    	    	char Hex_Str_128[129];//64byte to str
//	    	    	    	    	if(uart2_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart2_rx_buffer_ready = 0;
//	    	    	    	    		volatile uint8_t crc_1 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    	    	    		crc_1 ^= U2_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U2_rx[15] = crc_1;
//	    	    	    	    		if ((U2_rx[14] > 19)&&(U2_rx[14] < 30))//channel 2X
//	    	    	    	    		{
//	    	    	    	    			for (uint8_t i = 0; i < 16; i++) {
//	    	    	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U2_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str);
//	    	    	    	    		Send_AT_CMD("\r\n");
//	    	    	    	    		}
//
//	    	    	    	    	}
//
//	    	    	    	    	if(uart3_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart3_rx_buffer_ready = 0;
//	    	    	    	    		volatile uint8_t crc_2 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    	    	    		crc_2 ^= U3_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U3_rx[15] = crc_2;
//	    	    	    	    		if ((U3_rx[14] > 29)&&(U3_rx[14] < 40))//channel 3X
//	    	    	    	    		{
//	    	    	    	    			for (uint8_t i = 0; i < 16; i++) {
//	    	    	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U3_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str);
//	    	    	    	    		Send_AT_CMD("\r\n");
//	    	    	    	    		}
//	    	    	    	    	}
//
//	    	    	    	    	if(uart4_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart4_rx_buffer_ready = 0;
//	    	    	    	    		volatile uint8_t crc_3 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    	    	    		crc_3 ^= U4_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U4_rx[15] = crc_3;
//	    	    	    	    		if ((U4_rx[14] > 39)&&(U4_rx[14] < 50))//channel 4X
//	    	    	    	    		{
//
//	    	    	    	    			for (uint8_t i = 0; i < 16; i++) {
//	    	    	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U4_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str);
//	    	    	    	    		Send_AT_CMD("\r\n");
//
//	    	    	    	    		}
//	    	    	    	    	}
//
//	    	    	    	    	if(uart5_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart5_rx_buffer_ready = 0;
//	    	    	    	    		volatile uint8_t crc_4 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    	    	    		crc_4 ^= U5_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U5_rx[15] = crc_4;
//	    	    	    	    		if ((U5_rx[14] > 49)&&(U5_rx[14] < 60))//channel 5X
//	    	    	    	    		{
//
//	    	    	    	    			for (uint8_t i = 0; i < 16; i++) {
//	    	    	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U5_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str);
//	    	    	    	    		Send_AT_CMD("\r\n");
//
//	    	    	    	    		}
//	    	    	    	    	}
//
//	    	    	    	    	if(uart6_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart6_rx_buffer_ready = 0;
//
//	    	    	    	    		volatile uint8_t crc_5 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 15; z++) {
//	    	    	    	    		crc_5 ^= U6_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U6_rx[15] = crc_5;
//	    	    	    	    		if ((U6_rx[14] > 59)&&(U6_rx[14] < 70))//channel 6X
//	    	    	    	    		{
//	    	    	    	    			for (uint8_t i = 0; i < 16; i++) {
//	    	    	    	    			    sprintf(&Hex_Str[i * 2], "%02X", U6_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=32\r\n");//send 32byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str);
//	    	    	    	    		Send_AT_CMD("\r\n");
//	    	    	    	    		}
//	    	    	    	    	}
//
//	    	    	    	    	if(uart7_rx_buffer_ready == 1)
//	    	    	    	    	{
//	    	    	    	    		uart7_rx_buffer_ready = 0;
//	    	    	    	    		volatile uint8_t crc_0 = 0;
//	    	    	    	    		for (uint8_t z = 1; z < 63; z++) {
//	    	    	    	    		crc_0 ^= U7_rx[z];
//	    	    	    	    		}
//	    	    	    	    		U7_rx[63] = crc_0;
//	    	    	    	    		if (U7_rx[62] == 70)//only device id = 70
//	    	    	    	    		{
//	    	    	    	    			for (uint8_t i = 0; i < 64; i++) {
//	    	    	    	    			    sprintf(&Hex_Str_128[i * 2], "%02X", U7_rx[i]);
//	    	    	    	    			}
//	    	    	    	    		Send_AT_CMD("AT+CIPSEND=128\r\n");//send 128byte
//	    	    	    	    		HAL_Delay(10);
//	    	    	    	    		Send_AT_CMD(Hex_Str_128);
//	    	    	    	    		Send_AT_CMD("\r\n");
//	    	    	    	    		}
//
//	    	    	    	    	}
//   	    	}
   	    	//WIFI end
  }//while end
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D22571;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 460800;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 460800;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 460800;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 460800;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_TX_CL_GPIO_Port, RS485_TX_CL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin RS485_TX_CL_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|RS485_TX_CL_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_0_Pin */
  GPIO_InitStruct.Pin = INT1_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_1_Pin */
  GPIO_InitStruct.Pin = INT1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB12
                           PB13 PB14 PB15 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
