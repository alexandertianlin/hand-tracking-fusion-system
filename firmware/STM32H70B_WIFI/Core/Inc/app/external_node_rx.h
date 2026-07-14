#ifndef EXTERNAL_NODE_RX_H
#define EXTERNAL_NODE_RX_H

#include "main.h"
#include "imu/imu_config.h"

typedef enum {
  EXTERNAL_NODE_PORT_NONE = 0U,
  EXTERNAL_NODE_PORT_UART2 = 2U,
  EXTERNAL_NODE_PORT_UART3 = 3U,
  EXTERNAL_NODE_PORT_UART4 = 4U,
  EXTERNAL_NODE_PORT_UART5 = 5U,
  EXTERNAL_NODE_PORT_UART6 = 6U,
  EXTERNAL_NODE_PORT_UART7 = 7U
} external_node_port_t;

typedef struct {
  uint8_t source_port;
  uint8_t frame[PALM_PROTOCOL_FRAME_SIZE];
} external_node_frame_t;

void external_node_rx_init(void);
void external_node_rx_ingest_byte(external_node_port_t port_id, uint8_t byte);
uint8_t external_node_rx_has_frame(void);
HAL_StatusTypeDef external_node_rx_pop_frame(external_node_frame_t *frame);

/* Palm-side runtime "Zero All" reference layer for forwarded fingertip nodes.
 * Request: arm a capture on every port; the next valid frame per port is
 * recorded as its zero reference, and subsequent frames are rebased so the
 * output quaternion reads (inv(ref) * q_mapped).
 * Clear: drop references and resume pass-through behavior. */
void external_node_rx_request_zero_all(void);
void external_node_rx_clear_zero_all(void);

#endif /* EXTERNAL_NODE_RX_H */
