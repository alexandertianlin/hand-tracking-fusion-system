/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "app/external_node_rx.h"
#include "imu/motionfx_wrapper.h"
#include "usb_process.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t Count_temp3 = 0;  //Uart3 RX total Count
volatile uint8_t Count_temp4 = 0;  //Uart4 RX total Count
volatile uint8_t Count_temp5 = 0;  //Uart5 RX total Count
volatile uint8_t Count_temp6 = 0;  //Uart6 RX total Count
volatile uint8_t Count_temp2 = 0;  //Uart2 RX total Count
volatile uint8_t Count_temp7 = 0;  //uart7 RX total Count

volatile uint8_t U2_rx_buffer[USART_REC_LEN];     //Total uart2 Max RX buffer = 80
volatile uint8_t U3_rx_buffer[USART_REC_LEN];     //Total uart3 Max RX buffer = 80
volatile uint8_t U4_rx_buffer[USART_REC_LEN];     //Total uart4 Max RX buffer = 80
volatile uint8_t U5_rx_buffer[USART_REC_LEN];     //Total uart5 Max RX buffer = 80
volatile uint8_t U6_rx_buffer[USART_REC_LEN];     //Total uart6 Max RX buffer = 80
volatile uint8_t U7_rx_buffer[USART_REC_LEN];     //Total uart7 Max RX buffer = 80

uint8_t Temp2_Header;
uint8_t Temp3_Header;
uint8_t Temp4_Header;
uint8_t Temp5_Header;
uint8_t Temp6_Header;
uint8_t Temp7_Header;

volatile uint8_t U2_receive_len;//effective packet count target = 16byte
volatile uint8_t U3_receive_len;//effective packet count target = 16byte
volatile uint8_t U4_receive_len;//effective packet count target = 16byte
volatile uint8_t U5_receive_len;//effective packet count target = 16byte
volatile uint8_t U6_receive_len;//effective packet count target = 16byte
volatile uint8_t U7_receive_len_64;//effective packet count target = 64byte

volatile uint8_t U2_flag;
volatile uint8_t U3_flag;
volatile uint8_t U4_flag;
volatile uint8_t U5_flag;
volatile uint8_t U6_flag;
volatile uint8_t U7_flag;

extern uint8_t U2_rx[Protocol_len];     //uart2 packet 16byte
extern uint8_t U3_rx[Protocol_len];     //uart2 packet 16byte
extern uint8_t U4_rx[Protocol_len];     //uart2 packet 16byte
extern uint8_t U5_rx[Protocol_len];     //uart2 packet 16byte
extern uint8_t U6_rx[Protocol_len];     //uart2 packet 16byte
extern uint8_t U7_rx[Protocol_len_64];     //uart7 packet 64byte

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void fault_blink_stage(uint32_t stage)
{
  volatile uint32_t delay;
  uint32_t pulse;

  if (stage == 0U) {
    stage = 1U;
  }

  while (1)
  {
    for (pulse = 0U; pulse < stage; pulse++) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      for (delay = 0U; delay < 1800000U; delay++) {
        __NOP();
      }

      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      for (delay = 0U; delay < 1800000U; delay++) {
        __NOP();
      }
    }

    for (delay = 0U; delay < 5000000U; delay++) {
      __NOP();
    }
  }
}

uint8_t lpuart1_rx_buffer;//uart_data_out
uint8_t uart1_rx_buffer;
uint8_t uart2_rx_buffer;
uint8_t uart3_rx_buffer;
uint8_t uart4_rx_buffer;
uint8_t uart5_rx_buffer;
uint8_t uart6_rx_buffer;
uint8_t uart7_rx_buffer;

 uint8_t uart3_rx_buffer_ready;
 uint8_t uart4_rx_buffer_ready;
 uint8_t uart5_rx_buffer_ready;
 uint8_t uart6_rx_buffer_ready;
 uint8_t uart2_rx_buffer_ready;
 uint8_t uart7_rx_buffer_ready;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef lphuart1;
/* USER CODE BEGIN EV */

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
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  fault_blink_stage(g_motionfx_diag_stage);

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
  fault_blink_stage(g_motionfx_diag_stage);

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
  fault_blink_stage(g_motionfx_diag_stage);

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
  fault_blink_stage(g_motionfx_diag_stage);

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
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
      while (HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1) != HAL_OK) {
      }
    }
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
  if(HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY)
  {
   		while (HAL_UART_Receive_IT(&huart2, &uart2_rx_buffer, 1) != HAL_OK) {}//
  }
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
    while (HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1) != HAL_OK) {
    }
  }
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  if (HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) {
    while (HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1) != HAL_OK) {
    }
  }
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  if (HAL_UART_GetState(&huart5) == HAL_UART_STATE_READY) {
    while (HAL_UART_Receive_IT(&huart5, &uart5_rx_buffer, 1) != HAL_OK) {
    }
  }
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY) {
    while (HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer, 1) != HAL_OK) {
    }
  }
  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */

  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */
  if (HAL_UART_GetState(&huart7) == HAL_UART_STATE_READY) {
    while (HAL_UART_Receive_IT(&huart7, &uart7_rx_buffer, 1) != HAL_OK) {
    }
  }
  /* USER CODE END UART7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint8_t calc_crc(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

// receive callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    // ====================== UART2 ======================
    if (huart->Instance == USART2)
    {
        Count_temp2++;
        U2_rx_buffer[Count_temp2-1] = uart2_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART2, uart2_rx_buffer);

        // 1. 寻找包头阶段 (未锁定)
        if (U2_flag == 0)
        {
            if (uart2_rx_buffer == Temp2_Header)
            {
                U2_flag = 1; // 锁定！
                U2_receive_len = 0;
                U2_rx[U2_receive_len++] = uart2_rx_buffer;
            }
        }
        // 2. 接收数据阶段 (已锁定，闭着眼睛存满)
        else if (U2_flag == 1)
        {
            U2_rx[U2_receive_len++] = uart2_rx_buffer;

            // 3. 收满 Protocol_len (35字节) 开始校验
            if (U2_receive_len == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                // 动态计算 CRC，支持任意 Protocol_len
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U2_rx[z];
                }

                if (crc1 == U2_rx[Protocol_len - 1]) // 校验最后一个字节
                {
                    uart2_rx_buffer_ready = 1;
                    U2_rx[14] = U2_rx[14] + 20; // 0x14
                }
                // 结束这帧，解锁并重置，准备找下一帧
                Count_temp2 = 0;
                U2_receive_len = 0;
                U2_flag = 0;
            }
        }

        if (Count_temp2 > USART_REC_LEN)
        {
            Count_temp2 = 0;
            U2_receive_len = 0;
            U2_flag = 0;
        }

        HAL_UART_Receive_IT(&huart2, &uart2_rx_buffer, 1);
    }
    // ====================== UART3 ======================
    else if (huart->Instance == USART3)
    {
        Count_temp3++;
        U3_rx_buffer[Count_temp3-1] = uart3_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART3, uart3_rx_buffer);

        if (U3_flag == 0)
        {
            if (uart3_rx_buffer == Temp3_Header)
            {
                U3_flag = 1;
                U3_receive_len = 0;
                U3_rx[U3_receive_len++] = uart3_rx_buffer;
            }
        }
        else if (U3_flag == 1)
        {
            U3_rx[U3_receive_len++] = uart3_rx_buffer;
            if (U3_receive_len == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U3_rx[z];
                }
                if (crc1 == U3_rx[Protocol_len - 1])
                {
                    uart3_rx_buffer_ready = 1;
                    U3_rx[14] = U3_rx[14] + 30; // 0x1E
                }
                Count_temp3 = 0;
                U3_receive_len = 0;
                U3_flag = 0;
            }
        }

        if (Count_temp3 > USART_REC_LEN)
        {
            Count_temp3 = 0;
            U3_receive_len = 0;
            U3_flag = 0;
        }

        HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
    }
    // ====================== UART4 ======================
    else if (huart->Instance == UART4)
    {
        Count_temp4++;
        U4_rx_buffer[Count_temp4-1] = uart4_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART4, uart4_rx_buffer);

        if (U4_flag == 0)
        {
            if (uart4_rx_buffer == Temp4_Header)
            {
                U4_flag = 1;
                U4_receive_len = 0;
                U4_rx[U4_receive_len++] = uart4_rx_buffer;
            }
        }
        else if (U4_flag == 1)
        {
            U4_rx[U4_receive_len++] = uart4_rx_buffer;
            if (U4_receive_len == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U4_rx[z];
                }
                if (crc1 == U4_rx[Protocol_len - 1])
                {
                    uart4_rx_buffer_ready = 1;
                    U4_rx[14] = U4_rx[14] + 40; // 0x28
                }
                Count_temp4 = 0;
                U4_receive_len = 0;
                U4_flag = 0;
            }
        }

        if (Count_temp4 > USART_REC_LEN)
        {
            Count_temp4 = 0;
            U4_receive_len = 0;
            U4_flag = 0;
        }

        HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1);
    }
    // ====================== UART5 ======================
    else if (huart->Instance == UART5)
    {
        Count_temp5++;
        U5_rx_buffer[Count_temp5-1] = uart5_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART5, uart5_rx_buffer);

        if (U5_flag == 0)
        {
            if (uart5_rx_buffer == Temp5_Header)
            {
                U5_flag = 1;
                U5_receive_len = 0;
                U5_rx[U5_receive_len++] = uart5_rx_buffer;
            }
        }
        else if (U5_flag == 1)
        {
            U5_rx[U5_receive_len++] = uart5_rx_buffer;
            if (U5_receive_len == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U5_rx[z];
                }
                if (crc1 == U5_rx[Protocol_len - 1])
                {
                    uart5_rx_buffer_ready = 1;
                    U5_rx[14] = U5_rx[14] + 50; // 0x32
                }
                Count_temp5 = 0;
                U5_receive_len = 0;
                U5_flag = 0;
            }
        }

        if (Count_temp5 > USART_REC_LEN)
        {
            Count_temp5 = 0;
            U5_receive_len = 0;
            U5_flag = 0;
        }

        HAL_UART_Receive_IT(&huart5, &uart5_rx_buffer, 1);
    }
    // ====================== UART6 ======================
    else if (huart->Instance == USART6)
    {
        Count_temp6++;
        U6_rx_buffer[Count_temp6-1] = uart6_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART6, uart6_rx_buffer);

        if (U6_flag == 0)
        {
            if (uart6_rx_buffer == Temp6_Header)
            {
                U6_flag = 1;
                U6_receive_len = 0;
                U6_rx[U6_receive_len++] = uart6_rx_buffer;
            }
        }
        else if (U6_flag == 1)
        {
            U6_rx[U6_receive_len++] = uart6_rx_buffer;
            if (U6_receive_len == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U6_rx[z];
                }
                if (crc1 == U6_rx[Protocol_len - 1])
                {
                    uart6_rx_buffer_ready = 1;
                    U6_rx[14] = U6_rx[14] + 60; // 0x3C
                }
                Count_temp6 = 0;
                U6_receive_len = 0;
                U6_flag = 0;
            }
        }

        if (Count_temp6 > USART_REC_LEN)
        {
            Count_temp6 = 0;
            U6_receive_len = 0;
            U6_flag = 0;
        }

        HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer, 1);
    }
    // ====================== UART7 (小拇指修复：统一 35 字节) ======================
    else if (huart->Instance == UART7)
    {
        Count_temp7++;
        U7_rx_buffer[Count_temp7-1] = uart7_rx_buffer;
        external_node_rx_ingest_byte(EXTERNAL_NODE_PORT_UART7, uart7_rx_buffer);

        if (U7_flag == 0)
        {
            if (uart7_rx_buffer == Temp7_Header)
            {
                U7_flag = 1;
                U7_receive_len_64 = 0;
                U7_rx[U7_receive_len_64++] = uart7_rx_buffer;
            }
        }
        else if (U7_flag == 1)
        {
            U7_rx[U7_receive_len_64++] = uart7_rx_buffer;


            if (U7_receive_len_64 == Protocol_len)
            {
                volatile uint8_t crc1 = 0;
                for (uint8_t z = 1; z < (Protocol_len - 1); z++) {
                    crc1 ^= U7_rx[z];
                }

                if (crc1 == U7_rx[Protocol_len - 1])
                {
                    uart7_rx_buffer_ready = 1;
                    U7_rx[14] = U7_rx[14] + 70;
                }

                Count_temp7 = 0;
                U7_receive_len_64 = 0;
                U7_flag = 0;
            }
        }

        if (Count_temp7 > USART_REC_LEN)
        {
            Count_temp7 = 0;
            U7_receive_len_64 = 0;
            U7_flag = 0;
        }

        HAL_UART_Receive_IT(&huart7, &uart7_rx_buffer, 1);
    }
}
/* USER CODE END 1 */
