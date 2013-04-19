/*
 * co_packet : utilities for parsing CANopen communication objects
 * author : Mike Hopkins
 */

#include <stdio.h>
#include <string.h>
#include "co_packet.h"

static inline long min(long a, long b) {return a < b ? a : b;}
static inline long max(long a, long b) {return a > b ? a : b;}

static const uint32_t CO_TPDO[] = {CO_TPDO1, CO_TPDO2, CO_TPDO3, CO_TPDO4};
static const uint32_t CO_RPDO[] = {CO_RPDO1, CO_RPDO2, CO_RPDO3, CO_RPDO4};

struct can_frame co_packet::nmt(uint8_t node_id, uint8_t cs)
{
  struct can_frame frame;
  frame.can_id = CO_NMT;
  frame.can_dlc = 2;
  memcpy((char *)(frame.data), &cs, 1);
  memcpy((char *)(frame.data) + 1, &node_id, 1);
  memset((char *)(frame.data) + 2, 0, 6);
  return frame;
}

struct can_frame co_packet::sync(uint8_t counter)
{
  struct can_frame frame;
  frame.can_id = CO_SYNC;
  frame.can_dlc = 1;
  memcpy((char *)(frame.data), &counter, 1);
  memset((char *)(frame.data) + 1, 0, 7);
  return frame;
}

struct can_frame co_packet::emcy(uint8_t node_id, uint16_t error_code, 
uint8_t error_register, const char *data, size_t len)
{
  struct can_frame frame;
  frame.can_id = CO_EMCY | node_id;
  frame.can_dlc = min(3 + len, 8);
  memcpy((char *)(frame.data), &error_code, 2);
  memcpy((char *)(frame.data) + 2, &error_register, 1);
  memset((char *)(frame.data) + 3, 0, 5);
  if(data != NULL) 
    memcpy((uint8_t *)(frame.data) + 3, data, min(len, 5));
  return frame;
}

struct can_frame co_packet::time(uint32_t ms, uint16_t days)
{
  struct can_frame frame;
  frame.can_id = CO_TIME;
  frame.can_dlc = 6;
  ms = ms << 4;
  memcpy((char *)(frame.data), &ms, 4);
  memcpy((char *)(frame.data) + 4, &days, 2);
  memset((char *)(frame.data) + 6, 0, 2);
  return frame;
}

struct can_frame co_packet::rpdo(uint8_t node_id, uint8_t rpdo_number,
const char* data, size_t len)
{
  struct can_frame frame;
  rpdo_number = rpdo_number - 1;
  if(rpdo_number > 3)
    perror("Invalid PDO number");
  frame.can_id = CO_RPDO[rpdo_number] | node_id;
  frame.can_dlc = min(len, 8);
  memset((char *)(frame.data), 0, 8);
  if(data != NULL)
    memcpy((char *)(frame.data), data, min(len, 8));
  return frame;
}

struct can_frame co_packet::tpdo(uint8_t node_id, uint8_t tpdo_number,
const char* data, size_t len)
{
  struct can_frame frame;
  tpdo_number = tpdo_number - 1;
  if(tpdo_number > 3)
    perror("Invalid PDO number");
  frame.can_id = CO_TPDO[tpdo_number] | node_id;
  frame.can_dlc = min(len, 8);
  memset((char *)(frame.data), 0, 8);
  if(data != NULL)
    memcpy((char *)(frame.data), data, min(len, 8));
  return frame;
}

struct can_frame co_packet::tpdo_request(uint8_t node_id, uint8_t tpdo_number,
size_t len)
{
  struct can_frame frame;
  tpdo_number = tpdo_number - 1;
  if(tpdo_number > 3)
    perror("Invalid PDO number");
  frame.can_id = CO_TPDO[tpdo_number] | node_id | CAN_RTR_FLAG;
  frame.can_dlc = min(len, 8);
  memset((char *)(frame.data), 0, 8);
  return frame;
}

struct can_frame co_packet::rsdo(uint8_t node_id, uint16_t index, uint8_t subindex,
uint8_t cs, const char* data, size_t len)
{
  /* does not support segmented sdo's */
  struct can_frame frame;
  uint8_t n = min(max(4 - len, 0), 3);
  uint8_t header = (cs << 5);
  if(len > 0) header |= (n << 2) | 3;
  frame.can_id = CO_RSDO | node_id;
  frame.can_dlc = 8;
  memcpy((char *)(frame.data), &header, 1);
  memcpy((char *)(frame.data) + 1, &index, 2);
  memcpy((char *)(frame.data) + 3, &subindex, 1);
  memset((char *)(frame.data) + 4, 0, 4);
  if(data != NULL)
    memcpy((char *)(frame.data) + 4, data, min(len, 4));
  return frame;
}

struct can_frame co_packet::tsdo(uint8_t node_id, uint16_t index, uint8_t subindex,
uint8_t cs, const char* data, size_t len)
{
  /* does not support segmented sdo's */
  struct can_frame frame;
  uint8_t n = min(max(4 - len, 0), 3);
  uint8_t header = (cs << 5);
  if(len > 0) header |= (n << 2) | 3;
  frame.can_id = CO_TSDO | node_id;
  frame.can_dlc = 8;
  memcpy((char *)(frame.data), &header, 1);
  memcpy((char *)(frame.data) + 1, &index, 2);
  memcpy((char *)(frame.data) + 3, &subindex, 1);
  memset((char *)(frame.data) + 4, 0, 4);
  if(data != NULL)
    memcpy((char *)(frame.data) + 4, data, min(len, 4));
  return frame;
}

struct can_frame co_packet::nec(uint8_t node_id, uint8_t state)
{
  struct can_frame frame;
  frame.can_id = CO_NEC | node_id;
  frame.can_dlc = 1;
  memcpy((char *)(frame.data), &state, 1);
  memset((char *)(frame.data) + 1, 0, 7);
  return frame;
}

co_cob co_packet::parse_nmt(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&cob.cs, (char *)(frame->data), 1);
  memcpy(&cob.node, (char *)(frame->data) + 1, 1);
  return cob;
}

co_cob co_packet::parse_sync(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&cob.counter, (char *)(frame->data), 1);
  return cob;
}

co_cob co_packet::parse_emcy(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&cob.error_code, (char *)(frame->data), 2);
  memcpy(&cob.error_register, (char *)(frame->data) + 2, 1);
  memcpy(cob.data, (char *)(frame->data) + 3, 5);
  return cob;
}

co_cob co_packet::parse_time(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&cob.ms, (char *)(frame->data), 4);
  memcpy(&cob.days, (char *)(frame->data) + 4, 2);
  return cob;
}

co_cob co_packet::parse_pdo(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(cob.data, (char *)(frame->data), 8);
  return cob;
}

co_cob co_packet::parse_sdo(struct can_frame *frame)
{
  /* does not support segmented sdo's */

  uint8_t header;
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&header, (char *)(frame->data), 1);
  memcpy(&cob.index, (char *)(frame->data) + 1, 2);
  memcpy(&cob.subindex, (char *)(frame->data) + 3, 1);
  memcpy(cob.data, (char *)(frame->data) + 4, 4);
  cob.cs = (header & 0xE0) >> 5;
  return cob;
}

co_cob co_packet::parse_nec(struct can_frame *frame)
{
  co_cob cob;
  cob.cob_id  = frame->can_id & 0x7FF;
  cob.func_id = frame->can_id & 0x780;
  cob.node_id = frame->can_id & 0x07F;
  memcpy(&cob.state, (char *)(frame->data), 1);
  return cob;
}

co_cob co_packet::parse(struct can_frame *frame)
{
  uint32_t cob_id  = frame->can_id & 0x7FF;
  uint32_t func_id = frame->can_id & 0x780;
  switch (cob_id)
  {
    case CO_NMT :
      return parse_nmt(frame);
      break;
    case CO_SYNC :
      return parse_sync(frame);
      break;
    case CO_TIME :
      return parse_time(frame);
  }
  switch (func_id)
  {
    case CO_EMCY :
      return parse_emcy(frame);
      break;
    case CO_TSDO :
    case CO_RSDO :
      return parse_sdo(frame);
      break;
    case CO_NEC :
      return parse_nec(frame);
  }
  /* default */
  return parse_pdo(frame);
}
