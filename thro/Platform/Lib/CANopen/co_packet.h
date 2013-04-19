#ifndef CO_PACKET_H
#define CO_PACKET_H

/*
 * co_packet : utilities for parsing CANopen communication objects
 * author: Mike Hopkins
 */

#include <stdint.h>
#include <string.h>
#include "co_types.h"

namespace co_packet {
  struct can_frame nmt(uint8_t node_id, uint8_t cs);
  struct can_frame sync(uint8_t counter = 0);
  struct can_frame time(uint32_t ms, uint16_t days);
  struct can_frame emcy(uint8_t node_id, uint16_t error_code, 
    uint8_t error_register, const char *data = NULL, size_t len = 0);
  struct can_frame rpdo(uint8_t node_id, uint8_t rpdo_number,
    const char *data, size_t len);
  struct can_frame tpdo(uint8_t node_id, uint8_t tpdo_number,
    const char *data, size_t len);
  struct can_frame tpdo_request(uint8_t node_id, uint8_t tpdo_number,
    size_t len = 0);
  struct can_frame rsdo(uint8_t node_id, uint16_t index, uint8_t subindex,
    uint8_t cs, const char *data = NULL, size_t len = 0);
  struct can_frame tsdo(uint8_t node_id, uint16_t index,  uint8_t subindex,
    uint8_t cs, const char *data = NULL, size_t len = 0);
  struct can_frame nec(uint8_t node_id, uint8_t state);
  co_cob parse_nmt(struct can_frame *frame);
  co_cob parse_sync(struct can_frame *frame);
  co_cob parse_emcy(struct can_frame *frame);
  co_cob parse_time(struct can_frame *frame);
  co_cob parse_pdo(struct can_frame *frame);
  co_cob parse_sdo(struct can_frame *frame);
  co_cob parse_nec(struct can_frame *frame);
  co_cob parse(struct can_frame *frame);
};

#endif
