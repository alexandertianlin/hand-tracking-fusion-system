#include "app/external_node_rx.h"

#include <math.h>
#include <string.h>

#define PALM_EXTERNAL_NODE_PORT_COUNT 6U
#define PALM_EXTERNAL_NODE_INVALID_INDEX 0xFFU

typedef struct {
  uint8_t collecting;
  uint8_t length;
  uint8_t buffer[PALM_PROTOCOL_FRAME_SIZE];
} external_node_parser_t;

typedef struct {
  external_node_frame_t entries[PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH];
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile uint8_t count;
  volatile uint32_t dropped_frames;
} external_node_queue_t;

typedef struct {
  imu_quatf_t alignment;
  uint8_t multiply_order;
} external_node_quat_remap_config_t;

static external_node_parser_t g_external_node_parsers[PALM_EXTERNAL_NODE_PORT_COUNT];
static external_node_queue_t g_external_node_queue;

#if PALM_EXTERNAL_NODE_COMMON_MAP_ENABLE
static const imu_quatf_t g_external_node_common_map = {
  PALM_EXTERNAL_NODE_COMMON_MAP_W,
  PALM_EXTERNAL_NODE_COMMON_MAP_X,
  PALM_EXTERNAL_NODE_COMMON_MAP_Y,
  PALM_EXTERNAL_NODE_COMMON_MAP_Z
};
#endif

static imu_quatf_t g_external_node_zero_reference[PALM_EXTERNAL_NODE_PORT_COUNT];
static volatile uint8_t g_external_node_zero_pending[PALM_EXTERNAL_NODE_PORT_COUNT];
static volatile uint8_t g_external_node_zero_captured[PALM_EXTERNAL_NODE_PORT_COUNT];

static const external_node_quat_remap_config_t g_external_node_quat_remaps[PALM_EXTERNAL_NODE_PORT_COUNT] = {
  { { PALM_EXTERNAL_NODE_UART2_REMAP_W, PALM_EXTERNAL_NODE_UART2_REMAP_X,
      PALM_EXTERNAL_NODE_UART2_REMAP_Y, PALM_EXTERNAL_NODE_UART2_REMAP_Z },
    PALM_EXTERNAL_NODE_UART2_REMAP_ORDER },
  { { PALM_EXTERNAL_NODE_UART3_REMAP_W, PALM_EXTERNAL_NODE_UART3_REMAP_X,
      PALM_EXTERNAL_NODE_UART3_REMAP_Y, PALM_EXTERNAL_NODE_UART3_REMAP_Z },
    PALM_EXTERNAL_NODE_UART3_REMAP_ORDER },
  { { PALM_EXTERNAL_NODE_UART4_REMAP_W, PALM_EXTERNAL_NODE_UART4_REMAP_X,
      PALM_EXTERNAL_NODE_UART4_REMAP_Y, PALM_EXTERNAL_NODE_UART4_REMAP_Z },
    PALM_EXTERNAL_NODE_UART4_REMAP_ORDER },
  { { PALM_EXTERNAL_NODE_UART5_REMAP_W, PALM_EXTERNAL_NODE_UART5_REMAP_X,
      PALM_EXTERNAL_NODE_UART5_REMAP_Y, PALM_EXTERNAL_NODE_UART5_REMAP_Z },
    PALM_EXTERNAL_NODE_UART5_REMAP_ORDER },
  { { PALM_EXTERNAL_NODE_UART6_REMAP_W, PALM_EXTERNAL_NODE_UART6_REMAP_X,
      PALM_EXTERNAL_NODE_UART6_REMAP_Y, PALM_EXTERNAL_NODE_UART6_REMAP_Z },
    PALM_EXTERNAL_NODE_UART6_REMAP_ORDER },
  { { PALM_EXTERNAL_NODE_UART7_REMAP_W, PALM_EXTERNAL_NODE_UART7_REMAP_X,
      PALM_EXTERNAL_NODE_UART7_REMAP_Y, PALM_EXTERNAL_NODE_UART7_REMAP_Z },
    PALM_EXTERNAL_NODE_UART7_REMAP_ORDER }
};

static uint8_t external_node_rx_crc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

static int16_t external_node_rx_read_i16_le(const uint8_t *src)
{
  uint16_t value = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
  return (int16_t)value;
}

static void external_node_rx_write_i16_le(uint8_t *dst, int16_t value)
{
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
}

static int16_t external_node_rx_clamp_to_i16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)value;
}

static imu_quatf_t external_node_rx_quat_multiply(imu_quatf_t a, imu_quatf_t b)
{
  imu_quatf_t res;
  res.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  res.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  res.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  res.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return res;
}

static imu_quatf_t external_node_rx_quat_normalize(imu_quatf_t q)
{
  float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (norm <= 0) {
    q.w = 1; q.x = q.y = q.z = 0;
    return q;
  }
  q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
  if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
  return q;
}

// ==============================
// ✅ 新协议：四元数从 8 开始
// ==============================
static void external_node_rx_decode_quaternion(const uint8_t *frame, imu_quatf_t *q)
{
  q->w = (float)external_node_rx_read_i16_le(&frame[8])  / PALM_QUAT_SCALE;
  q->x = (float)external_node_rx_read_i16_le(&frame[10]) / PALM_QUAT_SCALE;
  q->y = (float)external_node_rx_read_i16_le(&frame[12]) / PALM_QUAT_SCALE;
  q->z = (float)external_node_rx_read_i16_le(&frame[14]) / PALM_QUAT_SCALE;
}

static void external_node_rx_encode_quaternion(uint8_t *frame, imu_quatf_t q)
{
  q = external_node_rx_quat_normalize(q);
  external_node_rx_write_i16_le(&frame[8],  external_node_rx_clamp_to_i16(q.w * PALM_QUAT_SCALE));
  external_node_rx_write_i16_le(&frame[10], external_node_rx_clamp_to_i16(q.x * PALM_QUAT_SCALE));
  external_node_rx_write_i16_le(&frame[12], external_node_rx_clamp_to_i16(q.y * PALM_QUAT_SCALE));
  external_node_rx_write_i16_le(&frame[14], external_node_rx_clamp_to_i16(q.z * PALM_QUAT_SCALE));
}

static uint8_t external_node_rx_port_index(external_node_port_t port_id)
{
  switch (port_id) {
    case EXTERNAL_NODE_PORT_UART2: return 0;
    case EXTERNAL_NODE_PORT_UART3: return 1;
    case EXTERNAL_NODE_PORT_UART4: return 2;
    case EXTERNAL_NODE_PORT_UART5: return 3;
    case EXTERNAL_NODE_PORT_UART6: return 4;
    case EXTERNAL_NODE_PORT_UART7: return 5;
    default: return PALM_EXTERNAL_NODE_INVALID_INDEX;
  }
}

static uint8_t external_node_rx_port_base_id(external_node_port_t port_id)
{
  switch (port_id) {
    case EXTERNAL_NODE_PORT_UART2: return PALM_EXTERNAL_NODE_UART2_BASE_ID;
    case EXTERNAL_NODE_PORT_UART3: return PALM_EXTERNAL_NODE_UART3_BASE_ID;
    case EXTERNAL_NODE_PORT_UART4: return PALM_EXTERNAL_NODE_UART4_BASE_ID;
    case EXTERNAL_NODE_PORT_UART5: return PALM_EXTERNAL_NODE_UART5_BASE_ID;
    case EXTERNAL_NODE_PORT_UART6: return PALM_EXTERNAL_NODE_UART6_BASE_ID;
    case EXTERNAL_NODE_PORT_UART7: return PALM_EXTERNAL_NODE_UART7_BASE_ID;
    default: return 0;
  }
}

static imu_quatf_t external_node_rx_apply_remap(const external_node_quat_remap_config_t *cfg, imu_quatf_t in)
{
  if (cfg->multiply_order == PALM_EXTERNAL_NODE_REMAP_RIGHT_MULTIPLY)
    return external_node_rx_quat_multiply(in, cfg->alignment);
  return external_node_rx_quat_multiply(cfg->alignment, in);
}

static imu_quatf_t external_node_rx_quat_conj(imu_quatf_t q)
{
  q.x = -q.x; q.y = -q.y; q.z = -q.z; return q;
}

static imu_quatf_t external_node_rx_apply_output_remap(imu_quatf_t in)
{
#if PALM_OUTPUT_FRAME_REMAP_ENABLE
  imu_quatf_t remap = {PALM_OUTPUT_FRAME_REMAP_W, PALM_OUTPUT_FRAME_REMAP_X,
                        PALM_OUTPUT_FRAME_REMAP_Y, PALM_OUTPUT_FRAME_REMAP_Z};
  remap = external_node_rx_quat_normalize(remap);
  imu_quatf_t conj = external_node_rx_quat_conj(remap);
  imu_quatf_t out = external_node_rx_quat_multiply(remap, in);
  out = external_node_rx_quat_multiply(out, conj);
  return external_node_rx_quat_normalize(out);
#else
  return external_node_rx_quat_normalize(in);
#endif
}

static HAL_StatusTypeDef external_node_rx_remap_orientation(external_node_port_t port_id, uint8_t *frame)
{
  uint8_t idx = external_node_rx_port_index(port_id);
  if (idx == PALM_EXTERNAL_NODE_INVALID_INDEX) return HAL_ERROR;
  const external_node_quat_remap_config_t *cfg = &g_external_node_quat_remaps[idx];

  imu_quatf_t in;
  external_node_rx_decode_quaternion(frame, &in);
  in = external_node_rx_quat_normalize(in);

  imu_quatf_t out = external_node_rx_apply_remap(cfg, in);

#if PALM_EXTERNAL_NODE_COMMON_MAP_ENABLE
  imu_quatf_t common_conj = external_node_rx_quat_conj(g_external_node_common_map);
  out = external_node_rx_quat_multiply(g_external_node_common_map, out);
  out = external_node_rx_quat_multiply(out, common_conj);
#endif

  out = external_node_rx_apply_output_remap(out);

  if (g_external_node_zero_pending[idx]) {
    g_external_node_zero_reference[idx] = out;
    g_external_node_zero_captured[idx] = 1;
    g_external_node_zero_pending[idx] = 0;
  }
  if (g_external_node_zero_captured[idx]) {
    imu_quatf_t ref_conj = external_node_rx_quat_conj(g_external_node_zero_reference[idx]);
    out = external_node_rx_quat_multiply(ref_conj, out);
    out = external_node_rx_quat_normalize(out);
  }

  external_node_rx_encode_quaternion(frame, out);
  return HAL_OK;
}

static void external_node_rx_reset_parser(external_node_parser_t *p)
{
  p->collecting = 0;
  p->length = 0;
}

// ==============================
// ✅ 新协议：node_id = 6
// ==============================
static HAL_StatusTypeDef external_node_rx_remap_node_id(external_node_port_t port_id, uint8_t *frame)
{
  uint8_t base = external_node_rx_port_base_id(port_id);
  uint8_t local = frame[6];
  if (base == 0) return HAL_ERROR;

  uint8_t mapped = (local <= PALM_EXTERNAL_NODE_MAX_LOCAL_ID) ? (base + local) : local;
  frame[6] = mapped;

  // ✅ CRC 重新计算（最后一位）
  frame[PALM_PROTOCOL_FRAME_SIZE - 1] = external_node_rx_crc(frame, PALM_PROTOCOL_FRAME_SIZE - 1);
  return HAL_OK;
}

static void external_node_rx_enqueue_frame(external_node_port_t port_id, const uint8_t *frame)
{
  uint8_t w = g_external_node_queue.head;
  if (g_external_node_queue.count >= PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH) {
    g_external_node_queue.tail = (g_external_node_queue.tail + 1) % PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH;
    g_external_node_queue.count--;
    g_external_node_queue.dropped_frames++;
  }

  external_node_frame_t *e = &g_external_node_queue.entries[w];
  e->source_port = port_id;
  memcpy(e->frame, frame, PALM_PROTOCOL_FRAME_SIZE);

  if (external_node_rx_remap_orientation(port_id, e->frame) != HAL_OK) { g_external_node_queue.dropped_frames++; return; }
  if (external_node_rx_remap_node_id(port_id, e->frame) != HAL_OK)   { g_external_node_queue.dropped_frames++; return; }

  g_external_node_queue.head = (w + 1) % PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH;
  g_external_node_queue.count++;
}

void external_node_rx_init(void)
{
  memset(g_external_node_parsers, 0, sizeof(g_external_node_parsers));
  memset(&g_external_node_queue, 0, sizeof(g_external_node_queue));
  for (uint8_t i = 0; i < PALM_EXTERNAL_NODE_PORT_COUNT; i++) {
    g_external_node_zero_reference[i].w = 1;
    g_external_node_zero_reference[i].x = g_external_node_zero_reference[i].y = g_external_node_zero_reference[i].z = 0;
    g_external_node_zero_pending[i] = 0;
    g_external_node_zero_captured[i] = 0;
  }
}

void external_node_rx_request_zero_all(void)
{
  for (uint8_t i = 0; i < PALM_EXTERNAL_NODE_PORT_COUNT; i++) {
    g_external_node_zero_captured[i] = 0;
    g_external_node_zero_pending[i] = 1;
  }
}

void external_node_rx_clear_zero_all(void)
{
  for (uint8_t i = 0; i < PALM_EXTERNAL_NODE_PORT_COUNT; i++) {
    g_external_node_zero_pending[i] = 0;
    g_external_node_zero_captured[i] = 0;
    g_external_node_zero_reference[i].w = 1;
    g_external_node_zero_reference[i].x = g_external_node_zero_reference[i].y = g_external_node_zero_reference[i].z = 0;
  }
}

// ==============================
// ✅ 新协议：三帧头 B5 A5 55
// ==============================
void external_node_rx_ingest_byte(external_node_port_t port_id, uint8_t byte)
{
  if (!PALM_EXTERNAL_NODE_RX_ENABLE) return;

  uint8_t idx = external_node_rx_port_index(port_id);
  if (idx == PALM_EXTERNAL_NODE_INVALID_INDEX) return;

  external_node_parser_t *p = &g_external_node_parsers[idx];

  if (!p->collecting) {
    if (byte == 0xB5) {
      external_node_rx_reset_parser(p);
      p->buffer[0] = byte;
      p->length = 1;
      p->collecting = 1;
    }
    return;
  }

  if (p->length >= PALM_PROTOCOL_FRAME_SIZE) {
    external_node_rx_reset_parser(p);
    if (byte == 0xB5) {
      p->buffer[0] = byte;
      p->length = 1;
      p->collecting = 1;
    }
    return;
  }

  p->buffer[p->length++] = byte;

  // 同步第二、第三帧头
  if (p->length == 2 && p->buffer[1] != 0xA5) { external_node_rx_reset_parser(p); return; }
  if (p->length == 3 && p->buffer[2] != 0x55) { external_node_rx_reset_parser(p); return; }

  if (p->length < PALM_PROTOCOL_FRAME_SIZE) return;

  // ✅ 新协议 CRC 校验
  uint8_t crc_calc = external_node_rx_crc(p->buffer, PALM_PROTOCOL_FRAME_SIZE - 1);
  uint8_t crc_rx = p->buffer[PALM_PROTOCOL_FRAME_SIZE - 1];

  if (crc_calc == crc_rx) {
    external_node_rx_enqueue_frame(port_id, p->buffer);
  }

  external_node_rx_reset_parser(p);
}

uint8_t external_node_rx_has_frame(void) {
  return g_external_node_queue.count > 0 ? 1 : 0;
}

HAL_StatusTypeDef external_node_rx_pop_frame(external_node_frame_t *frame)
{
  if (!frame || g_external_node_queue.count == 0) return HAL_ERROR;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t r = g_external_node_queue.tail;
  memcpy(frame, &g_external_node_queue.entries[r], sizeof(*frame));
  g_external_node_queue.tail = (r + 1) % PALM_EXTERNAL_NODE_RX_QUEUE_DEPTH;
  g_external_node_queue.count--;

  if (!primask) __enable_irq();
  return HAL_OK;
}
