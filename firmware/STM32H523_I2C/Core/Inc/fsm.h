#ifndef __FSM_H__
#define __FSM_H__

#include "global.h"
#include "main.h"

// Main loop FSM
typedef enum {
    CMD_MODE,           // waiting for cmd form host, use CMD FSM
    READING_MODE,       // read data from mlx90393 and send system[] to host using loop
    ACK_MODE,            // send ack to host when the board is plugged in
	FLASH_MODE
} main_loop_state_t;

// CMD FSM
// Two different types of CMD, 1 byte and 3 bytes
// 1 bytes have 0xCC, 0xDD, 0xB4, 0xF0
// for 0xCC and 0xDD, simply send C/D-serial num
// 0xB4 is to enter reading mode, 0xF0 is reset and enter cmd mode, 
// 0xF0 is async reset, whenever got 0xF0, stop everything and reset system[] and system_count
// Both 0xB4 and 0xF0 must send to next slave when received
// 3 bytes CMD start with 0x4E as starting byte
// second byte is board ID, when ID is 0, then this request is for this board
// third byte is CMD/address, refer to register table for more info
//typedef enum {
//    CMD_IDLE,
//    CMD_RECEIVE_1,      // determine if 1 byte or 3 bytes
//    CMD_RECEIVE_2,      // receive 2nd byte and verify id
//    CMD_RECEIVE_DONE    // receive 3rd byte, send corresponding request to host/ pass the cmd to next slave
//} cmd_state_t;


// Slave transmit Mode
typedef enum{
    TX_IDLE,           // shld never call transmit when idle
    TX_ACK_MODE,       // the message to be send is an ack message
    TX_CMD_MODE,       // the message to be send is a cmd message， only call when in CMD_MODE
    TX_READING_MODE    // the message to be send is a reading message
} slave_tx_mode_t;

// Standard FSM for UART receive 1 standard packet from other slaves
typedef enum {
    RX_WAIT_START,   // waiting for 0xB4/0xB5 as starting byte
    RX_RECEIVING,   // receiving data
    RX_WAIT_CRC,     // calculate CRC and varify if correct
    RX_RECEIVE_DONE  // receive done, carry on for packet process determination
} slave_rx_state_t;


// FSM for storing which mode the received packet is in, help determine keep the packet or not
typedef enum {
    RX_ACK_MODE,      // Got 0xB5 as starting byte
    RX_CMD_MODE,      // Status byte == 0xAA
    RX_NORMAL_MODE,     // Got 0xB4 as starting byte
    RX_TBD            // Waiting for start byte
} slave_rx_mode_t;


#endif
