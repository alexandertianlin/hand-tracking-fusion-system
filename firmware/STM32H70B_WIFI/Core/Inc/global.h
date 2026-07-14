#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "main.h"
#define Protocol_len 35	//protocol length 16bytes at uart2,3,4,5,6
#define Protocol_len_64 35	//protocol length 64bytes for SPI1x5 at uart7


//#define START_BYTE_1 0x81
//#define START_BYTE_2 0x5A
//#define END_BYTE_1 0xA5
//#define END_BYTE_2 0x81

#define USART_REC_LEN 80	//Max RX buffer length 80

// Speical Bytes
#define MAX_BOARDS 5
#define ACK_BYTE 0xB5//
#define SLAVE_ACK_STATUS 0x5B//
#define START_BYTE 0xB4//
#define CMD_STATUS_BYTE 0xAA//
#define HOST_ACK_BYTEX 0xFF
#define HOST_ACK_BYTE0 0x4E
#define RESET_BYTE 0xF0
#define MODEL_READING_CMD  0xB0
#define START_READING_CMD1 0xB1
#define START_READING_CMD2 0xB2
#define START_READING_CMD3 0xB3
#define C_Serial_READING_CMD 0xCC
#define D_Serial_READING_CMD 0xDD

/* 2-byte Zero All command group (palm-side only; fingertip firmware unchanged).
 * Sequence: { GLOVE_ZERO_CMD, GLOVE_ZERO_ALL_SUBCMD } arms a capture so the
 * next palm fused sample and next forwarded frame per port become the zero
 * reference. { GLOVE_ZERO_CMD, GLOVE_ZERO_CLEAR_SUBCMD } drops the
 * references and resumes raw output. */
#define GLOVE_ZERO_CMD          0xC0
#define GLOVE_ZERO_ALL_SUBCMD   0x01
#define GLOVE_ZERO_CLEAR_SUBCMD 0x00

/* Request values for the shared main-loop glove-zero service flag. */
#define GLOVE_ZERO_REQUEST_NONE  0U
#define GLOVE_ZERO_REQUEST_ZERO  1U
#define GLOVE_ZERO_REQUEST_CLEAR 2U
#endif
