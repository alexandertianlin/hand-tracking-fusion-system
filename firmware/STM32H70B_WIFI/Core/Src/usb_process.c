#include "usb_process.h"
#include "string.h"
#include "global.h"

#include "stm32h7xx_it.h"
#include "usbd_cdc_if.h"
#include "float.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;


extern uint8_t usb_receive_buffer[8];
extern uint8_t usb_receive_buffer_index;
extern uint8_t g_usb_receive_flag;

extern uint8_t g_LPUART1_data_out_flag;
extern uint8_t g_USB_data_out_flag;

extern uint8_t g_B2_flag;
extern uint8_t g_B3_flag;
extern uint8_t g_B4_flag;
extern uint8_t g_B5_flag;
extern uint8_t g_B6_flag;
extern uint8_t g_B7_flag;

extern volatile uint8_t g_glove_zero_request;



void Process_USB_Receive_Buffer() {
  // check len
	uint8_t cmd_byte;

  if (usb_receive_buffer_index == 0) {
    __atomic_store_n(&g_usb_receive_flag, 0, __ATOMIC_RELAXED);
    return;
  }
  else if (usb_receive_buffer_index == 1) {
	    __atomic_store_n(&g_usb_receive_flag, 0, __ATOMIC_RELAXED);
	    return;
	  }
  else if (usb_receive_buffer_index == 2)
  {//index = 2 start
 //PC USB IF connect send {0xFF,0xF0}
    if ((usb_receive_buffer[0] == HOST_ACK_BYTEX)&&(usb_receive_buffer[1] == RESET_BYTE))
    	{
    	 cmd_byte = HOST_ACK_BYTEX;//0xFF
    	 	if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x21};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
    	 	 HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x31};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x41};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x51};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x61};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
     		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
         	    	uint8_t test_rx[1] = {0x71};
     	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
       		}
     		HAL_Delay(10);


        	 cmd_byte = RESET_BYTE;//0xF0
    		if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        	    	uint8_t test_rx[1] = {0x22};
    	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
        		}
    			HAL_Delay(10);
        		if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        	    	uint8_t test_rx[1] = {0x32};
    	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
        		}
         		HAL_Delay(10);
        		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        	    	uint8_t test_rx[1] = {0x42};
    	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
        		}
         		HAL_Delay(10);
        		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        	    	uint8_t test_rx[1] = {0x52};
    	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
        		}
         		HAL_Delay(10);
        		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        	    	uint8_t test_rx[1] = {0x62};
    	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
        		}
         		HAL_Delay(10);
          		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
              	    	uint8_t test_rx[1] = {0x72};
          	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);

              		}
         		HAL_Delay(10);
    	}
    else if ((usb_receive_buffer[0] == MODEL_READING_CMD)&&(usb_receive_buffer[1] == START_READING_CMD1))
    	{
        __atomic_store_n(&g_B2_flag, 1, __ATOMIC_RELAXED);//uart2 on
        __atomic_store_n(&g_B3_flag, 1, __ATOMIC_RELAXED);//uart3 on
        __atomic_store_n(&g_B4_flag, 1, __ATOMIC_RELAXED);//uart4 on
        __atomic_store_n(&g_B5_flag, 1, __ATOMIC_RELAXED);//uart5 on
        __atomic_store_n(&g_B6_flag, 1, __ATOMIC_RELAXED);//uart6 on
        __atomic_store_n(&g_B7_flag, 1, __ATOMIC_RELAXED);//uart7 on

		 cmd_byte = MODEL_READING_CMD;
			if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x23};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
			if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x33};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x43};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x53};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x63};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
	 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	     	    	uint8_t test_rx[1] = {0x73};
	 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	     		}
	 		HAL_Delay(10);


			 cmd_byte = START_READING_CMD1;
				if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x24};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
				if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x34};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x44};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x54};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x64};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
		 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
		     	    	uint8_t test_rx[1] = {0x74};
		 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
		     		}
		 		HAL_Delay(10);
    	}


    else if ((usb_receive_buffer[0] == MODEL_READING_CMD)&&(usb_receive_buffer[1] == START_READING_CMD2))
    	{
        __atomic_store_n(&g_B2_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B3_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B4_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B5_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B6_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B7_flag, 1, __ATOMIC_RELAXED);




		 cmd_byte = MODEL_READING_CMD;

			if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x25};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
			if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x35};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x45};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x55};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x65};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
	 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	     	    	uint8_t test_rx[1] = {0x75};
	 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);

	     		}
	 		HAL_Delay(10);
			 cmd_byte = START_READING_CMD2;

				if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x26};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
				if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x36};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x46};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x56};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x66};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
		 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
		     	    	uint8_t test_rx[1] = {0x76};
		 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);

		     		}
		 		HAL_Delay(10);
    	}
    else if ((usb_receive_buffer[0] == MODEL_READING_CMD)&&(usb_receive_buffer[1] == START_READING_CMD3))
    	{
        __atomic_store_n(&g_B2_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B3_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B4_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B5_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B6_flag, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&g_B7_flag, 1, __ATOMIC_RELAXED);



		 cmd_byte = MODEL_READING_CMD;

			if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x27};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
			if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
				uint8_t test_rx[1] = {0x37};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
			}
	 		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x47};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x57};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
    	    	uint8_t test_rx[1] = {0x67};
	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    		}
     		HAL_Delay(10);
	 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	     	    	uint8_t test_rx[1] = {0x77};
	 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);

	     		}
	 		HAL_Delay(10);
			 cmd_byte = START_READING_CMD3;

				if (HAL_UART_Transmit(&huart2, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x28};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
				if (HAL_UART_Transmit(&huart3, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
					uint8_t test_rx[1] = {0x38};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
				}
		 		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart4, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x48};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart5, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x58};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
	    		if (HAL_UART_Transmit(&huart6, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	    	    	uint8_t test_rx[1] = {0x68};
		    		CDC_Transmit_HS((uint8_t*)test_rx, 1);
	    		}
	     		HAL_Delay(10);
		 		if (HAL_UART_Transmit(&huart7, &cmd_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
		     	    	uint8_t test_rx[1] = {0x78};
		 	    		CDC_Transmit_HS((uint8_t*)test_rx, 1);

		     		}
		 		HAL_Delay(10);
    	}
    else if ((usb_receive_buffer[0] == GLOVE_ZERO_CMD) && (usb_receive_buffer[1] == GLOVE_ZERO_ALL_SUBCMD))
    {
      /* Palm-side Zero All: defer the actual capture to the main loop via
       * a shared request flag; palm_runtime and external_node_rx will latch
       * their respective references on their next processed frames. */
      __atomic_store_n(&g_glove_zero_request, GLOVE_ZERO_REQUEST_ZERO, __ATOMIC_RELAXED);
      uint8_t test_rx[1] = {0xC1};
      CDC_Transmit_HS((uint8_t*)test_rx, 1);
    }
    else if ((usb_receive_buffer[0] == GLOVE_ZERO_CMD) && (usb_receive_buffer[1] == GLOVE_ZERO_CLEAR_SUBCMD))
    {
      __atomic_store_n(&g_glove_zero_request, GLOVE_ZERO_REQUEST_CLEAR, __ATOMIC_RELAXED);
      uint8_t test_rx[1] = {0xC0};
      CDC_Transmit_HS((uint8_t*)test_rx, 1);
    }
    else
    {
    	uint8_t test_rx[1] = {0x99};//2 byte cmd fail error
		CDC_Transmit_HS((uint8_t*)test_rx, 1);
    }

    	usb_receive_buffer_index = 0;
    	memset(usb_receive_buffer, 0, sizeof(usb_receive_buffer));
    	__atomic_store_n(&g_usb_receive_flag, 0, __ATOMIC_RELAXED);
    	return;
  }//index = 2 end
  else//index > 2 start
  {
	  usb_receive_buffer_index = 0;
	  memset(usb_receive_buffer, 0, sizeof(usb_receive_buffer));
	  __atomic_store_n(&g_usb_receive_flag, 0, __ATOMIC_RELAXED);
  }//index > 2 end

}
