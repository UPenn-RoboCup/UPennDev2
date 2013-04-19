/*
 * co_slave : object dictionary interface for ds301 compliant slave devices
 * author : Mike Hopkins
 */

#include "co_slave.h"

/* defines standard ds301 object dictionary entries */
static co_dictionary_entry co_slave_dictionary[] = {
  {CO_DEVICE_TYPE, 0x00, CO_UNSIGNED32},
  {CO_ERROR_REGISTER, 0x00, CO_UNSIGNED8},
  {CO_STORE_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_STORE_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_STORE_PARAMETERS, 0x02, CO_UNSIGNED32},
  {CO_STORE_PARAMETERS, 0x03, CO_UNSIGNED32},
  {CO_RESTORE_DEFAULT_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_RESTORE_DEFAULT_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_RESTORE_DEFAULT_PARAMETERS, 0x02, CO_UNSIGNED32},
  {CO_RESTORE_DEFAULT_PARAMETERS, 0x03, CO_UNSIGNED32},
  {CO_IDENTITY_OBJECT, 0x00, CO_UNSIGNED8},
  {CO_IDENTITY_OBJECT, 0x01, CO_UNSIGNED32},
  {CO_IDENTITY_OBJECT, 0x02, CO_UNSIGNED32},
  {CO_IDENTITY_OBJECT, 0x03, CO_UNSIGNED32},
  {CO_IDENTITY_OBJECT, 0x04, CO_UNSIGNED32},
  {CO_IDENTITY_OBJECT, 0x05, CO_UNSIGNED32},
  {CO_RPDO1_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_RPDO1_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_RPDO1_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_RPDO1_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_RPDO1_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_RPDO1_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_RPDO2_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_RPDO2_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_RPDO2_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_RPDO2_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_RPDO2_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_RPDO2_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_RPDO3_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_RPDO3_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_RPDO3_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_RPDO3_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_RPDO3_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_RPDO3_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_RPDO4_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_RPDO4_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_RPDO4_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_RPDO4_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_RPDO4_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_RPDO4_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_TPDO1_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_TPDO1_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_TPDO1_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_TPDO1_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_TPDO1_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_TPDO1_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_TPDO2_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_TPDO2_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_TPDO2_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_TPDO2_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_TPDO2_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_TPDO2_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_TPDO3_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_TPDO3_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_TPDO3_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_TPDO3_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_TPDO3_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_TPDO3_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_TPDO4_PARAMETERS, 0x00, CO_UNSIGNED8},
  {CO_TPDO4_PARAMETERS, 0x01, CO_UNSIGNED32},
  {CO_TPDO4_PARAMETERS, 0x02, CO_UNSIGNED8},
  {CO_TPDO4_PARAMETERS, 0x03, CO_UNSIGNED16},
  {CO_TPDO4_PARAMETERS, 0x04, CO_UNSIGNED8},
  {CO_TPDO4_PARAMETERS, 0x05, CO_UNSIGNED16},
  {CO_RPDO1_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_RPDO1_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_RPDO1_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_RPDO2_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_RPDO2_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_RPDO3_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_RPDO3_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_RPDO4_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_RPDO4_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_TPDO1_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_TPDO1_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_TPDO2_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_TPDO2_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_TPDO3_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_TPDO3_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x00, CO_UNSIGNED8},
  {CO_TPDO4_MAPPING, 0x01, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x02, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x03, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x04, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x05, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x06, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x07, CO_UNSIGNED32},
  {CO_TPDO4_MAPPING, 0x08, CO_UNSIGNED32},
  {CO_SENTINEL}
};

co_slave::co_slave(uint8_t node_id) : 
m_node_id(node_id), m_state(0), m_error_register(0), m_error_code(0)
{
  /* initialize object dictionary */
  register_dictionary_entries(co_slave_dictionary);
  set_value(CO_RPDO1_MAPPING, 0x00, 0);
  set_value(CO_RPDO2_MAPPING, 0x00, 0);
  set_value(CO_RPDO3_MAPPING, 0x00, 0);
  set_value(CO_RPDO4_MAPPING, 0x00, 0);
  set_value(CO_TPDO1_MAPPING, 0x00, 0);
  set_value(CO_TPDO2_MAPPING, 0x00, 0);
  set_value(CO_TPDO3_MAPPING, 0x00, 0);
  set_value(CO_TPDO4_MAPPING, 0x00, 0);
}

uint8_t co_slave::get_node_id()
{
  return m_node_id;
}

uint8_t co_slave::get_state()
{
  return m_state;
}

uint8_t co_slave::get_error_register()
{
  return m_error_register;
}

uint16_t co_slave::get_error_code()
{
  return m_error_code;
}

void co_slave::set_node_id(uint8_t node_id)
{
  m_node_id = node_id;
}

void co_slave::set_state(uint8_t state)
{
  m_state = state;
}

void co_slave::set_error_register(uint8_t error_register)
{
  m_error_register = error_register;
}

void co_slave::set_error_code(uint16_t error_code)
{
  m_error_code = error_code;
}

bool co_slave::register_dictionary_entries(const co_dictionary_entry entries[])
{
  /* add entries to local object dictionary */
  bool success = true;
  const co_dictionary_entry *entry = entries;
  while (1)
  {
    if (entry->index == CO_SENTINEL)
      break;
    success = set_entry(entry) && success;
    entry++;
  }
  return success;
}

bool co_slave::register_sdo_settings(const co_sdo_setting settings[])
{
  /* register object dictionary settings for sdo configuration */
  bool success = true;
  const co_sdo_setting *setting = settings;
  while (1)
  {
     if(setting->index == CO_SENTINEL)
       break;
     success = set_value(setting->index, setting->subindex, setting->value) && success;
     setting++;
  }
  return success;
}

bool co_slave::register_pdo_parameters(const co_pdo_parameter parameters[])
{
  /* register pdo parameters for pdo configuration*/
  const co_pdo_parameter *parameter = parameters;
  while (1)
  {
    if (parameter->cob_id == CO_SENTINEL)
      break;
    /* get pdo parameter index */
    uint16_t parameter_index;
    switch (parameter->cob_id & 0x7F0)
    {
      case CO_RPDO1 :
        parameter_index = CO_RPDO1_PARAMETERS;
        break;
      case CO_RPDO2 :
	parameter_index = CO_RPDO2_PARAMETERS;
	break;
      case CO_RPDO3 :
        parameter_index = CO_RPDO3_PARAMETERS;
        break;
      case CO_RPDO4 :
        parameter_index = CO_RPDO4_PARAMETERS;
        break;
      case CO_TPDO1 :
        parameter_index = CO_TPDO1_PARAMETERS;
        break;
      case CO_TPDO2 :
        parameter_index = CO_TPDO2_PARAMETERS;
        break;
      case CO_TPDO3 :
        parameter_index = CO_TPDO3_PARAMETERS;
        break;
      case CO_TPDO4 :
        parameter_index = CO_TPDO4_PARAMETERS;
        break;
      default:
        return false;
    }
    /* update pdo parameter entries in object dictionary */
    uint32_t cob_id = (0xFFFFFF80 & parameter->cob_id) + m_node_id;
    set_data(parameter_index, 0x01, &cob_id);
    set_data(parameter_index, 0x02, &(parameter->transmission_type));
    set_data(parameter_index, 0x03, &(parameter->inhibit_time));
    set_data(parameter_index, 0x05, &(parameter->event_timer));
    parameter++;
  }
  return true;
}

bool co_slave::register_pdo_mappings(const co_pdo_mapping mappings[])
{
  /* register pdo mappings for pdo configuration */
  const co_pdo_mapping *mapping = mappings;
  while (1)
  {
    if(mapping->func_id == CO_SENTINEL)
      break;
    /* get mapping index for pdo */
    uint16_t mapping_index;
    switch (mapping->func_id)
    {
      case CO_RPDO1 :
        mapping_index = CO_RPDO1_MAPPING;
        break;
      case CO_RPDO2 :
        mapping_index = CO_RPDO2_MAPPING;
        break;
      case CO_RPDO3 :
        mapping_index = CO_RPDO3_MAPPING;
        break;
      case CO_RPDO4 :
        mapping_index = CO_RPDO4_MAPPING;
        break;
      case CO_TPDO1 :
        mapping_index = CO_TPDO1_MAPPING;
        break;
      case CO_TPDO2 :
        mapping_index = CO_TPDO2_MAPPING;
        break;
      case CO_TPDO3 :
        mapping_index = CO_TPDO3_MAPPING;
        break;
      case CO_TPDO4 :
        mapping_index = CO_TPDO4_MAPPING;
        break;
      default:
        return false;
    }
    for (int i = 0; i < mapping->n_entries; i++)
    {
      /* get dictionary entry for current mapping */
      uint16_t entry_index = (mapping->entries[i] & 0xFFFF0000) >> 16;
      uint16_t entry_subindex = (mapping->entries[i] & 0x0000FF00) >> 8;
      co_dictionary_entry *entry = get_entry(entry_index, entry_subindex);
      if (entry == NULL)
         return false;
      else
      {
        /* update pdo mapping in dictionary */
        uint8_t entry_size = co_data_size[entry->data_type];
        uint32_t data = entry_index << 16 | entry_subindex << 8 | entry_size << 3;
        set_data(mapping_index, i + 1, &data);
      }
    }
    /* update number of pdo mappings in dictionary */
    set_data(mapping_index, 0x00, &(mapping->n_entries));
    mapping++;
  }
  return true; 
}

void co_slave::set_sdo_connection(co_sdo_connection connection)
{
  m_sdo_connection = connection;
}

co_sdo_connection co_slave::get_sdo_connection()
{
  return m_sdo_connection;
}

int co_slave::get_sdo_status()
{
  return m_sdo_connection.status;
}

int co_slave::get_sdo_abort_code()
{
  return m_sdo_connection.abort_code;
}

int co_slave::set_pdo(uint32_t func_id, char pdo_data[8])
{
  /* load pdo data into object dictionary */
  uint16_t mapping_index;
  switch (func_id)
  {
    case CO_RPDO1 :
      mapping_index = CO_RPDO1_MAPPING;
      break;
    case CO_RPDO2 :
      mapping_index = CO_RPDO2_MAPPING;
      break;
    case CO_RPDO3 :
      mapping_index = CO_RPDO3_MAPPING;
      break;
    case CO_RPDO4 :
      mapping_index = CO_RPDO4_MAPPING;
      break;
    case CO_TPDO1 :
      mapping_index = CO_TPDO1_MAPPING;
      break;
    case CO_TPDO2 :
      mapping_index = CO_TPDO2_MAPPING;
      break;
    case CO_TPDO3 :
      mapping_index = CO_TPDO3_MAPPING;
      break;
    case CO_TPDO4 :
      mapping_index = CO_TPDO4_MAPPING;
      break;
    default:
      return 0;
  }

  int pdo_size = 0;
  int n_entries = get_value(mapping_index, 0x00);
  for (int i = 0; i < n_entries; i++)
  {
    /* get dictionary entry for current mapping */
    uint32_t mapping_data;
    get_data(mapping_index, i + 1, &mapping_data);
    uint8_t entry_size = (mapping_data & 0x000000FF) >> 3;
    uint16_t entry_index = (mapping_data & 0xFFFF0000) >> 16;
    uint8_t entry_subindex = (mapping_data & 0x0000FF00) >> 8;
    co_dictionary_entry *entry = get_entry(entry_index, entry_subindex);
    /* load pdo data into dictionary entry's data buffer */
    if (entry != NULL)
      memcpy(entry->process_data, pdo_data + pdo_size, entry_size);
    pdo_size += entry_size;
  }
  return pdo_size;
}

int co_slave::get_pdo(uint32_t func_id, char pdo_data[8])
{
  /* load object dictionary data into pdo */
  uint16_t mapping_index;
  switch (func_id)
  {
    case CO_RPDO1 :
      mapping_index = CO_RPDO1_MAPPING;
      break;
    case CO_RPDO2 :
      mapping_index = CO_RPDO2_MAPPING;
      break;
    case CO_RPDO3 :
      mapping_index = CO_RPDO3_MAPPING;
      break;
    case CO_RPDO4 :
      mapping_index = CO_RPDO4_MAPPING;
      break;
    case CO_TPDO1 :
      mapping_index = CO_TPDO1_MAPPING;
      break;
    case CO_TPDO2 :
      mapping_index = CO_TPDO2_MAPPING;
      break;
    case CO_TPDO3 :
      mapping_index = CO_TPDO3_MAPPING;
      break;
    case CO_TPDO4 :
      mapping_index = CO_TPDO4_MAPPING;
      break;
    default:
      return 0;
  }

  int pdo_size = 0;
  int n_entries = get_value(mapping_index, 0x00);
  for (int i = 0; i < n_entries; i++)
  {
    /* get dictionary entry for current mapping */
    uint32_t mapping_data;
    get_data(mapping_index, i + 1, &mapping_data);
    uint8_t entry_size = (mapping_data & 0x000000FF) >> 3;
    uint16_t entry_index = (mapping_data & 0xFFFF0000) >> 16;
    uint8_t entry_subindex = (mapping_data & 0x0000FF00) >> 8;
    co_dictionary_entry *entry = get_entry(entry_index, entry_subindex);
    /* load dictionary entry data into pdo's data buffer */
    if (entry != NULL)
      memcpy(pdo_data + pdo_size, entry->process_data, entry_size);
    pdo_size += entry_size;
  }
  return pdo_size;
}

void* co_slave::get_data_pointer(uint16_t index, uint8_t subindex)
{
  co_dictionary_entry *entry = get_entry(index, subindex);
  if (entry == NULL)
    return NULL;
  return entry->process_data;
}

int co_slave::set_data(uint16_t index, uint8_t subindex, const void *data)
{
  /* copy void *data buffer to dictionary data */
  co_dictionary_entry *entry = get_entry(index, subindex);
  if (entry == NULL)
    return 0;
  int entry_size = co_data_size[entry->data_type];
  memcpy(entry->process_data, data, entry_size);
  return entry_size;
}

int co_slave::get_data(uint16_t index, uint8_t subindex, void *data)
{
  /* copy dictionary data to void *data buffer */
  co_dictionary_entry *entry = get_entry(index, subindex);
  if (entry == NULL)
    return 0;
  int entry_size = co_data_size[entry->data_type];
  memcpy(data, entry->process_data, entry_size);
  return entry_size;
}

int co_slave::set_value(uint16_t index, uint8_t subindex, long double value)
{
  /* copy double value to dictionary data */
  co_dictionary_entry *entry = get_entry(index, subindex);
  if (entry == NULL)
    return 0;
  char data[8];
  memset(data, 0, 8);
  switch (entry->data_type)
  {
    case CO_UNSIGNED8 :
      *(uint8_t *)data = (uint8_t)value;
      break;
    case CO_UNSIGNED16 :
      *(uint16_t *)data = (uint16_t)value;
      break;
    case CO_UNSIGNED24 :
    case CO_UNSIGNED32 :
      *(uint32_t *)data = (uint32_t)value;
      break;
    case CO_UNSIGNED40 :
    case CO_UNSIGNED48 :
    case CO_UNSIGNED56 :
    case CO_UNSIGNED64 :
      *(uint64_t *)data = (uint64_t)value;
      break;
    case CO_INTEGER8 :
      *(int8_t *)data = (int8_t)value;
      break;
    case CO_INTEGER16 :
      *(int16_t *)data = (int16_t)value;
      break;
    case CO_INTEGER24 :
    case CO_INTEGER32 :
      *(int32_t *)data = (int32_t)value;
      break;
    case CO_INTEGER40 :
    case CO_INTEGER48 :
    case CO_INTEGER56 :
    case CO_INTEGER64 :
      *(int64_t *)data = (int64_t)value;
      break;
    case CO_REAL32 : 
      *(float *)data = (float)value;
      break;
    case CO_REAL64 : 
      *(double *)data = (double)value;
      break;
    default :
      return 0;
  }
  int entry_size = co_data_size[entry->data_type];
  memcpy(entry->process_data, data, entry_size);
  return entry_size;
}

long double co_slave::get_value(uint16_t index, uint8_t subindex)
{
  /* copy dictionary data to double value */
  long double value = 0;
  co_dictionary_entry *entry = get_entry(index, subindex);
  if (entry == NULL)
    return 0;
  char data[8];
  memset(data, 0, 8);
  int entry_size = co_data_size[entry->data_type];
  memcpy(data, entry->process_data, entry_size);
  switch (entry->data_type)
  {
    case CO_UNSIGNED8 : 
      value = (long double)(*(uint8_t *)data);
      break;
    case CO_UNSIGNED16 : 
      value = (long double)(*(uint16_t *)data);
      break;
    case CO_UNSIGNED24 : 
    case CO_UNSIGNED32 : 
      value = (long double)(*(uint32_t *)data);
      break;
    case CO_UNSIGNED40 :
    case CO_UNSIGNED48 :
    case CO_UNSIGNED56 :
    case CO_UNSIGNED64 :
      value = (long double)(*(uint64_t *)data);
      break;
    case CO_INTEGER8 : 
      value = (long double)(*(int8_t *)data);
      break;
    case CO_INTEGER16 : 
      value = (long double)(*(int16_t *)data);
      break;
    case CO_INTEGER24 : 
    case CO_INTEGER32 : 
      value = (long double)(*(int32_t *)data);
      break;
    case CO_INTEGER40 :
    case CO_INTEGER48 :
    case CO_INTEGER56 :
    case CO_INTEGER64 :
      value = (long double)(*(int64_t  *)data);
      break;
    case CO_REAL32 : 
      value = (long double)(*(float *)data);
      break;
    case CO_REAL64 : 
      value = (long double)(*(double *)data);
      break;
  }
  return value;
}

co_dictionary_entry* co_slave::get_entry(uint16_t index, uint8_t subindex)
{
  /* return pointer to dictionary entry */
  uint32_t key = CO_ENTRY(index, subindex);
  co_dictionary_map::iterator it = m_object_dictionary.find(key);
  if (it == m_object_dictionary.end())
    return NULL;
  return it->second;
}

bool co_slave::set_entry(const co_dictionary_entry *entry)
{
  /* add entry to local object dictionary */
  uint32_t key = CO_ENTRY(entry->index, entry->subindex);
  co_dictionary_entry *private_entry = get_entry(entry->index, entry->subindex);
  if (private_entry == NULL)
  {
    private_entry = new co_dictionary_entry;
    m_object_dictionary[key] = private_entry;
  }
  memcpy(private_entry, entry, sizeof(co_dictionary_entry)); 
  memset(private_entry->process_data, 0, 8);
  return true;
}

co_slave::~co_slave()
{
  /* free memory allocated for each dictionary entry */
  co_dictionary_map::iterator it;
  for (it = m_object_dictionary.begin(); it != m_object_dictionary.end(); it++)
    delete it->second;
}
