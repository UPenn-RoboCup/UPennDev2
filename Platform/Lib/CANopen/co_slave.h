#ifndef CO_SLAVE_H
#define CO_SLAVE_H

/*
 * co_slave : base class for ds301 compliant slave devices
 * author : Mike Hopkins
 */

#include <map>
#include <stdint.h>
#include <string.h>
#include "co_types.h"

/* defines standard ds301 object dictionary indices */
#define CO_DEVICE_TYPE                0x1000
#define CO_ERROR_REGISTER             0x1001
#define CO_STORE_PARAMETERS           0x1010
#define CO_RESTORE_DEFAULT_PARAMETERS 0x1011
#define CO_IDENTITY_OBJECT            0x1018
#define CO_RPDO1_PARAMETERS           0x1400
#define CO_RPDO2_PARAMETERS           0x1401
#define CO_RPDO3_PARAMETERS           0x1402
#define CO_RPDO4_PARAMETERS           0x1403
#define CO_TPDO1_PARAMETERS           0x1800
#define CO_TPDO2_PARAMETERS           0x1801
#define CO_TPDO3_PARAMETERS           0x1802
#define CO_TPDO4_PARAMETERS           0x1803
#define CO_RPDO1_MAPPING              0x1600
#define CO_RPDO2_MAPPING              0x1601
#define CO_RPDO3_MAPPING              0x1602
#define CO_RPDO4_MAPPING              0x1603
#define CO_TPDO1_MAPPING              0x1A00
#define CO_TPDO2_MAPPING              0x1A01
#define CO_TPDO3_MAPPING              0x1A02
#define CO_TPDO4_MAPPING              0x1A03

typedef std::map<uint32_t, co_dictionary_entry*> co_dictionary_map;

class co_slave {
private:
  uint8_t m_node_id;
  uint8_t m_state;
  uint8_t m_error_register;
  uint16_t m_error_code;
  co_sdo_connection m_sdo_connection;
  co_dictionary_map m_object_dictionary;
  bool set_entry(const co_dictionary_entry *entry);
  co_dictionary_entry* get_entry(uint16_t index, uint8_t subindex);
public:
  co_slave(uint8_t node_id = 1);
  uint8_t get_node_id();
  uint8_t get_state();
  uint8_t get_error_register();
  uint16_t get_error_code();
  void set_node_id(uint8_t node_id);
  void set_state(uint8_t state);
  void set_error_register(uint8_t error_register);
  void set_error_code(uint16_t error_code);
  bool register_dictionary_entries(const co_dictionary_entry entries[]);
  bool register_sdo_settings(const co_sdo_setting settings[]);
  bool register_pdo_parameters(const co_pdo_parameter parameters[]);
  bool register_pdo_mappings(const co_pdo_mapping mappings[]);
  void set_sdo_connection(co_sdo_connection connection);
  co_sdo_connection get_sdo_connection();
  int get_sdo_status();
  int get_sdo_abort_code();
  int set_pdo(uint32_t func_id, char pdo_data[8]);
  int get_pdo(uint32_t func_id, char pdo_data[8]);
  void *get_data_pointer(uint16_t index, uint8_t subindex);
  int set_data(uint16_t index, uint8_t subindex, const void *data);
  int get_data(uint16_t index, uint8_t subindex, void *data);
  int set_value(uint16_t index, uint8_t subindex, long double value);
  long double get_value(uint16_t index, uint8_t subindex);
  ~co_slave();
};

#endif
