#ifndef CO_TYPES_H
#define CO_TYPES_H

/*
 * co_types : constant and structure definitions for co_master lib 
 * author : Mike Hopkins
 */

#include <stdint.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* defines sentinal for struct arrays */
#define CO_SENTINEL 0

/* defines CANopen function ids */
#define CO_NMT   0x000
#define CO_SYNC  0x080
#define CO_EMCY  0x080
#define CO_TIME  0x100
#define CO_TPDO1 0x180
#define CO_RPDO1 0x200
#define CO_TPDO2 0x280
#define CO_RPDO2 0x300
#define CO_TPDO3 0x380
#define CO_RPDO3 0x400
#define CO_TPDO4 0x480
#define CO_RPDO4 0x500
#define CO_TSDO  0x580
#define CO_RSDO  0x600
#define CO_NEC   0x700

/* defines options for co_pdo_parameter */
#define CO_INVALID 0x80000000 // cob_id
#define CO_NO_RTR  0x40000000
#define CO_SYNCHRONOUS   0x00 // transmission_type
#define CO_ASYNCHRONOUS  0xFF
#define CO_MFR_SPECIFIC  0xFE
#define CO_RTR_UPDATE    0xFD
#define CO_SYNC_UPDATE   0xFC

/* defines macro to assign dictionary entries in co_pdo_mapping */
#define CO_ENTRY(index, subindex) ((index) << 16 | (subindex) << 8)

/* defines status flags for sdo connections */
enum co_sdo_status {
  CO_SDO_SUCCEEDED,
  CO_SDO_ABORTED,
  CO_SDO_WAITING
};

/* defines supported primitive data types */
enum co_data_type {
  CO_UNSIGNED8,
  CO_UNSIGNED16,
  CO_UNSIGNED24,
  CO_UNSIGNED32,
  CO_UNSIGNED40,
  CO_UNSIGNED48,
  CO_UNSIGNED56,
  CO_UNSIGNED64,
  CO_INTEGER8,
  CO_INTEGER16,
  CO_INTEGER24,
  CO_INTEGER32,
  CO_INTEGER40,
  CO_INTEGER48,
  CO_INTEGER56,
  CO_INTEGER64,
  CO_REAL32,
  CO_REAL64,
  CO_MAX_DATA_TYPE
};

/* defines primitive data type sizes */
extern const size_t co_data_size[CO_MAX_DATA_TYPE];

/* defines data type for can ids */
typedef uint32_t co_can_id;

/* defines structure for assigning slave sdo settings */
typedef struct co_sdo_setting {
  uint16_t index;
  uint8_t subindex;
  double value;
} co_sdo_setting;

/* defines structure for assigning slave pdo parameters */
typedef struct co_pdo_parameter {
  uint32_t cob_id;
  uint8_t transmission_type;
  uint16_t inhibit_time;
  uint16_t event_timer;
} co_pdo_parameter;

/* defines structure for assigning slave pdo mappings */
typedef struct co_pdo_mapping {
  uint32_t func_id;     // function id
  uint8_t n_entries;    // 0-8
  uint32_t entries[8];  // {CO_ENTRY(index, subindex), ...}
} co_pdo_mapping;

/* defines structure for tracking slave sdo connections */
typedef struct co_sdo_connection {
  uint16_t index;
  uint8_t subindex;
  uint8_t cs;
  co_sdo_status status;
  uint32_t abort_code;
} co_sdo_connection;

/* defines structure for slave object dictionary entries */
typedef struct co_dictionary_entry {
  uint16_t index;
  uint8_t subindex;
  uint8_t data_type;
  char *process_data;
  char service_data[8];
} co_dictionary_entry;

/* defines structure for communication objects */
typedef struct co_cob {
  /* required data */
  uint32_t cob_id;
  uint32_t func_id;
  uint32_t node_id;
  /* optional data */
  char data[8];
  uint16_t index;
  uint8_t subindex;
  uint8_t cs;
  uint8_t node;
  uint8_t state;
  uint8_t counter;
  uint16_t error_code;
  uint8_t error_register;
  uint32_t ms;
  uint16_t days;
} co_cob;

#endif
