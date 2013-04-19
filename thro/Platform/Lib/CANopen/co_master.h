#ifndef CO_MASTER_H
#define CO_MASTER_H

/*
 * co_master : simple CANopen master class
 * author : Mike Hopkins
 */

#include <map>
#include <string>
#include "co_types.h"
#include "co_slave.h"
#include "can_channel.h"

#define CO_MAX_SLAVES 127
#define DEFAULT_TIMEOUT -2

typedef std::map<co_can_id, struct can_frame *> co_can_mailbox;

class co_master {
private:
  bool m_debug_print;
  double m_default_timeout;
  void (*m_emcy_callback)(int, void *);
  void *m_emcy_user_data;
  can_channel *m_can_channel;
  co_slave *m_slave[CO_MAX_SLAVES + 1];
  co_can_mailbox m_can_mailbox;
public:
  co_master(can_channel *channel = NULL);
  can_channel *get_can_channel();
  void enable_debug_print(bool enable);
  void set_can_channel(can_channel *channel);
  void set_default_timeout(double timeout);
  void register_slave(co_slave *slave);
  void register_can_buffer(struct can_frame *buffer);
  void register_emcy_callback(
    void (*emcy_callback)(int, void *), void *user_data);
  void register_receive_callback(co_can_id can_id,
    void (*receive_callback)(void *), void *user_data);
  bool configure_pdo_settings(uint8_t node_id,
    double timeout = DEFAULT_TIMEOUT);
  bool configure_sdo_settings(uint8_t node_id, const co_sdo_setting settings[],
    double timeout = DEFAULT_TIMEOUT);

  int start_remote_node(uint8_t node_id);
  int stop_remote_node(uint8_t node_id);
  int enter_preoperational_state(uint8_t node_id);
  int reset_node(uint8_t node_id);
  int reset_communication(uint8_t node_id);
  int send_nmt(uint8_t node_id, uint8_t cs);
  int send_sync(uint8_t counter = 0);
  int send_time(uint32_t ms, uint16_t days);
  int send_emcy(uint8_t node_id, uint16_t error_code, uint8_t error_register,
    const char *data = NULL, size_t len = 0);
  int send_rpdo(uint8_t node_id, uint8_t rpdo_number);
  int send_tpdo_request(uint8_t node_id, uint8_t tpdo_number); 
  int start_sdo_download(uint8_t node_id, uint16_t index, uint8_t subindex);
  int start_sdo_upload(uint8_t node_id, uint16_t index, uint8_t subindex);
  int send_sdo_abort(uint8_t node_id, uint16_t index, uint8_t subindex);
  bool sdo_upload(uint8_t node_id, uint16_t index, uint8_t subindex,
    double timeout = DEFAULT_TIMEOUT);
  bool sdo_download(uint8_t node_id, uint16_t index, uint8_t subindex,
    double timeout = DEFAULT_TIMEOUT);
  bool restore_all_default_values(uint8_t node_id,
    double timeout = DEFAULT_TIMEOUT);
  bool store_all_parameters(uint8_t node_id, double timeout = DEFAULT_TIMEOUT);
  int send_nec(uint8_t node_id, uint8_t state);
  int send_can_frame(struct can_frame *frame);

  void flush_can_channel();
  bool handle_can_frame(struct can_frame *frame);
  bool handle_canopen_frame(struct can_frame *frame);
  bool handle_receive_callback(co_can_id can_id);
  bool receive(double timeout);
  bool receive(const co_can_id can_ids[], int n_can_ids,
    double timeout = DEFAULT_TIMEOUT);
};

#endif
