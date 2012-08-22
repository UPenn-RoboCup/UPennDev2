/*
 * co_master : Simple CANopen master class
 * author : Mike Hopkins
 */

#include <time.h>
#include <vector>
#include <stdio.h>
#include "co_master.h"
#include "co_packet.h"

static const uint32_t CO_TPDO[] = {CO_TPDO1, CO_TPDO2, CO_TPDO3, CO_TPDO4};
static const uint32_t CO_RPDO[] = {CO_RPDO1, CO_RPDO2, CO_RPDO3, CO_RPDO4};

static inline double get_time() 
{ 
  /* return monotonic system time in seconds */
  timespec ts;
#ifdef CLOCK_MONOTONIC_RAW
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return ts.tv_sec + ts.tv_nsec*1e-9;
}

/* CONFIG
 ************************************************************************/

co_master::co_master(can_channel *channel)
{
  m_can_channel = channel;
  for (int i = 0; i < CO_MAX_SLAVES; i++)
    m_slave[i] = NULL;
  m_debug_print = false;
  m_default_timeout = -1;
  m_emcy_callback = NULL;
}

void co_master::enable_debug_print(bool enable)
{
  m_debug_print = enable;
}

can_channel *co_master::get_can_channel()
{
  return m_can_channel;
}

void co_master::set_can_channel(can_channel *channel)
{
  m_can_channel = channel;
}

void co_master::set_default_timeout(double timeout)
{
  m_default_timeout = timeout;
}

void co_master::register_slave(co_slave *slave)
{
  /* register co_slave instance for slave communication */
  m_slave[slave->get_node_id()] = slave;
}

void co_master::register_can_buffer(struct can_frame *buffer)
{
  /* register receive buffer for storage of frames with matching can_id's */
  m_can_mailbox[buffer->can_id] = buffer;
}

void co_master::register_emcy_callback(
void (*emcy_callback)(int /* node_id */, void * /* user_data */), void *user_data)
{
  /* register function callback for incoming emcy messages */
  m_emcy_callback = emcy_callback;
  m_emcy_user_data = user_data;
}

void co_master::register_receive_callback(co_can_id can_id,
void (*receive_callback)(void * /* user_data */), void *user_data)
{
  /* TODO register function callbacks for incoming can_id's */
}

bool co_master::configure_pdo_settings(uint8_t node_id, double timeout)
{
  /* configure pdo settings for CANopen slave */
  if (m_slave[node_id] == NULL) 
    return false;
  uint16_t mapping_index[8] =
    {0x1600, 0x1601, 0x1602, 0x1603, 0x1A00, 0x1A01, 0x1A02, 0x1A03};
  uint16_t parameter_index[8] =
    {0x1400, 0x1401, 0x1402, 0x1403, 0x1800, 0x1801, 0x1802, 0x1803};
  bool success = true;
  for (int i = 0; i < 8; i++)
  {
    /* configure pdo parameters */
    success = sdo_download(node_id, parameter_index[i], 0x01, timeout) && success;
    success = sdo_download(node_id, parameter_index[i], 0x02, timeout) && success;
    if(!sdo_download(node_id, parameter_index[i], 0x03, timeout))
      success &= (m_slave[node_id]->get_sdo_connection().abort_code == 0x06090011);
    if(!sdo_download(node_id, parameter_index[i], 0x05, timeout))
      success &= (m_slave[node_id]->get_sdo_connection().abort_code == 0x06090011);
    /* configure pdo mappings */
    uint8_t n_entries = m_slave[node_id]->get_value(mapping_index[i], 0x00);
    m_slave[node_id]->set_value(mapping_index[i], 0x00, 0);
    success = sdo_download(node_id, mapping_index[i], 0x00, timeout) && success;
    for (int j = 0; j < n_entries; j++)
      success = sdo_download(node_id, mapping_index[i], j + 1, timeout) && success;
    m_slave[node_id]->set_value(mapping_index[i], 0x00, n_entries);
    success = sdo_download(node_id, mapping_index[i], 0x00, timeout) && success;
  }
  return success;
}

bool co_master::configure_sdo_settings(uint8_t node_id, const co_sdo_setting settings[], double timeout)
{
  /* configure sdo settings for CANopen slave */
  if (m_slave[node_id] == NULL) 
    return false;
  bool success = true;
  const co_sdo_setting *setting = settings;
  while (1)
  {
    if ((setting->index == CO_SENTINEL) || (!success))
      break;
    success = sdo_download(node_id, setting->index, setting->subindex, timeout) && success;
    setting++;
  }
  return success;
}

/* NMT
 ************************************************************************/

int co_master::start_remote_node(uint8_t node_id)
{
  /* send start remote node nmt message */
  struct can_frame frame = co_packet::nmt(node_id, 0x01);
  return m_can_channel->send(&frame);
}

int co_master::stop_remote_node(uint8_t node_id)
{
  /* send stop remote node nmt message */
  struct can_frame frame = co_packet::nmt(node_id, 0x02);
  return m_can_channel->send(&frame);
}

int co_master::enter_preoperational_state(uint8_t node_id)
{
  /* send enter preoperational state nmt message */
  struct can_frame frame = co_packet::nmt(node_id, 0x80);
  return m_can_channel->send(&frame);
}

int co_master::reset_node(uint8_t node_id)
{
  /* send reset node nmt message */
  struct can_frame frame = co_packet::nmt(node_id, 0x81);
  return m_can_channel->send(&frame);
}

int co_master::reset_communication(uint8_t node_id)
{
  /* send reset communication nmt message */
  struct can_frame frame = co_packet::nmt(node_id, 0x82);
  return m_can_channel->send(&frame);
}

int co_master::send_nmt(uint8_t node_id, uint8_t cs)
{
  /* send generic nmt message */
  struct can_frame frame = co_packet::nmt(node_id, cs);
  return m_can_channel->send(&frame);
}

/* SYNC
 ************************************************************************/

int co_master::send_sync(uint8_t counter)
{
  /* send SYNC message */
  struct can_frame frame = co_packet::sync(counter);
  return m_can_channel->send(&frame);
}

/* EMCY
 ************************************************************************/

int co_master::send_emcy(uint8_t node_id, uint16_t error_code,
uint8_t error_register, const char *data, size_t len)
{
  /* send emergency message */
  struct can_frame frame = co_packet::emcy(node_id, error_code, error_register, data);
  return m_can_channel->send(&frame);
}

/* TIME
 ************************************************************************/

int co_master::send_time(uint32_t ms, uint16_t days)
{
  /* send timestamp message */
  struct can_frame frame = co_packet::time(ms, days);
  return m_can_channel->send(&frame);
}

/* PDO 
 ************************************************************************/

int co_master::send_rpdo(uint8_t node_id, uint8_t rpdo_number)
{
  /* send receive pdo message to slave node */
  if (m_slave[node_id] == NULL)
    return -1;
  char data[8];
  int len = m_slave[node_id]->get_pdo(CO_RPDO[rpdo_number - 1], data);
  struct can_frame frame = co_packet::rpdo(node_id, rpdo_number, data, len);
  return m_can_channel->send(&frame);
}

int co_master::send_tpdo_request(uint8_t node_id, uint8_t tpdo_number)
{
  /* send transmit pdo request to slave node */
  if (m_slave[node_id] == NULL)
    return -1;
  struct can_frame frame = co_packet::tpdo_request(node_id, tpdo_number);
  return m_can_channel->send(&frame);
}

/* SDO
 ************************************************************************/

int co_master::start_sdo_download(uint8_t node_id, uint16_t index, uint8_t subindex)
{
  /* initiate sdo download to slave node */
  if (m_slave[node_id] == NULL)
    return -1;
  char data[8];
  int len = m_slave[node_id]->get_data(index, subindex, data);
  struct can_frame frame = co_packet::rsdo(node_id, index, subindex, 1, data, len);
  co_sdo_connection connection = {index, subindex, 3, CO_SDO_WAITING};
  m_slave[node_id]->set_sdo_connection(connection);
  return m_can_channel->send(&frame);
}

int co_master::start_sdo_upload(uint8_t node_id, uint16_t index, uint8_t subindex)
{
  /* initiate sdo upload from slave node */
  if (m_slave[node_id] == NULL)
    return -1;
  struct can_frame frame = co_packet::rsdo(node_id, index, subindex, 2);
  co_sdo_connection connection = {index, subindex, 2, CO_SDO_WAITING};
  m_slave[node_id]->set_sdo_connection(connection);
  return m_can_channel->send(&frame);
}

int co_master::send_sdo_abort(uint8_t node_id, uint16_t index, uint8_t subindex)
{
  /* send sdo abort message */
  if (m_slave[node_id] == NULL)
    return -1;
  struct can_frame frame = co_packet::rsdo(node_id, index, subindex, 4);
  return m_can_channel->send(&frame);
}

bool co_master::sdo_download(uint8_t node_id, uint16_t index, uint8_t subindex, double timeout)
{
  /* complete sdo download to slave node */
  if (start_sdo_download(node_id, index, subindex) == -1)
    return false;
  co_can_id sdo_response[] = {CO_TSDO | node_id};
  receive(sdo_response, 1, timeout);
  co_sdo_connection connection = m_slave[node_id]->get_sdo_connection();
  if (connection.status == CO_SDO_SUCCEEDED)
    return true;
  else if(connection.status == CO_SDO_WAITING && m_debug_print)
    fprintf(stderr, "sdo timeout: node %d, index %X, subindex %X\n", node_id, index, subindex);
  else if(connection.status == CO_SDO_ABORTED && m_debug_print)
    fprintf(stderr, "sdo abort: node %d, index %X, subindex %X, code %X\n", node_id, index, subindex, connection.abort_code);
  return false;
}

bool co_master::sdo_upload(uint8_t node_id, uint16_t index, uint8_t subindex, double timeout)
{
  /* complete sdo upload from slave node */
  if (start_sdo_upload(node_id, index, subindex) == -1)
    return false;
  co_can_id sdo_response[] = {CO_TSDO | node_id};
  receive(sdo_response, 1, timeout);
  co_sdo_connection connection = m_slave[node_id]->get_sdo_connection();
  if (connection.status == CO_SDO_SUCCEEDED)
    return true;
  else if(connection.status == CO_SDO_WAITING && m_debug_print)
    fprintf(stderr, "sdo timeout: node %d, index %X, subindex %X\n", node_id, index, subindex);
  else if(connection.status == CO_SDO_ABORTED && m_debug_print)
    fprintf(stderr, "sdo abort: node %d, index %X, subindex %X, code %X\n", node_id, index, subindex, connection.abort_code);
  return false;
}

bool co_master::restore_all_default_values(uint8_t node_id, double timeout)
{
  /* request slave node to restore all default parameters */ 
  if (m_slave[node_id] == NULL)
    return false;
  m_slave[node_id]->set_value(CO_RESTORE_DEFAULT_PARAMETERS, 0x01, 0x64616F6C);
  return sdo_download(node_id, CO_RESTORE_DEFAULT_PARAMETERS, 0x01, timeout);
}

bool co_master::store_all_parameters(uint8_t node_id, double timeout)
{
  /* request slave node to store all parameters */ 
  if (m_slave[node_id] == NULL)
    return false;
  m_slave[node_id]->set_value(CO_STORE_PARAMETERS, 0x01,0x65766173);
  return sdo_download(node_id, CO_STORE_PARAMETERS, 0x01, timeout);
}

/* NEC
 ************************************************************************/

int co_master::send_nec(uint8_t node_id, uint8_t state)
{
  /* send network error control message */
  struct can_frame frame = co_packet::nec(node_id, state);
  return m_can_channel->send(&frame);
}

/* CAN
 ************************************************************************/

int co_master::send_can_frame(struct can_frame *frame)
{
  /* send generic can frame */
  return m_can_channel->send(frame);
}

/* RECEIVE
 ************************************************************************/

void co_master::flush_can_channel()
{
  /* flush can channel receive buffer */
  return m_can_channel->flush();
}

bool co_master::handle_can_frame(struct can_frame *frame)
{
  /* handle incoming can frame */
  co_can_mailbox::iterator it = m_can_mailbox.find(frame->can_id & 0x7FF);
  if (it != m_can_mailbox.end())
  {
    /* if buffer is registered for can_id, load frame data into buffer */
    *(it->second) = *frame;
    return true;
  }
  return false;
}

bool co_master::handle_canopen_frame(struct can_frame *frame)
{
  /* handle incoming CANopen communication object */
  co_cob cob = co_packet::parse(frame);
  if (m_slave[cob.node_id] == NULL)
    return false;
  co_sdo_connection connection = m_slave[cob.node_id]->get_sdo_connection();
  switch (cob.func_id) {
    case CO_TPDO1 :
    case CO_TPDO2 :
    case CO_TPDO3 :
    case CO_TPDO4 :
      /* load tpdo data into slave process image */
      m_slave[cob.node_id]->set_pdo(cob.func_id, cob.data);
      break;
    case CO_TSDO :
      /* load sdo upload data into slave dictionary */
      if (cob.cs == 2)
        m_slave[cob.node_id]->set_data(cob.index, cob.subindex, cob.data);
      /* update slave connection status */
      if (cob.index == connection.index && cob.subindex == connection.subindex)
      {
        if (cob.cs == connection.cs)
          connection.status = CO_SDO_SUCCEEDED;
        else if (cob.cs == 4)
        {
          connection.status = CO_SDO_ABORTED;
          connection.abort_code = *(uint32_t *)cob.data;
        }
        m_slave[cob.node_id]->set_sdo_connection(connection);
      }
      break;
    case CO_EMCY :
      /* set emcy error codes in slave */
      m_slave[cob.node_id]->set_error_register(cob.error_register);
      m_slave[cob.node_id]->set_error_code(cob.error_code);
      if (m_emcy_callback != NULL)
        (*m_emcy_callback)(cob.node_id, m_emcy_user_data);
      break;
    case CO_NEC :
      /* set nec state in slave */
      m_slave[cob.node_id]->set_state(cob.state);
      break;
    default :
      return false;
  }
  return true;
}

bool co_master::handle_receive_callback(co_can_id can_id)
{
  /* TODO handle receive callbacks for incoming can_id's */
  return false;
}

bool co_master::receive(double timeout)
{
  /* process incoming can frames until timeout occurs */
  return receive(NULL, 0, timeout);
}

bool co_master::receive(const co_can_id can_ids[], int n_can_ids, double timeout)
{
  /* process incoming can frames until specified can_ids are received */
  double t0 = get_time();
  std::vector<co_can_id> receive_ids(can_ids, can_ids + n_can_ids);
  if (timeout == DEFAULT_TIMEOUT)
    timeout = m_default_timeout;
  m_can_channel->set_timeout(timeout);
  while (1)
  {
    /* receive can frames */
    can_frame frame;
    if (m_can_channel->receive(&frame) >= 0)
    {
      co_can_id can_id = frame.can_id & 0x7FF;
      /* handle incoming frame */
      if (!handle_can_frame(&frame))
        handle_canopen_frame(&frame);
      handle_receive_callback(can_id);
      /* check if can_id is in the receive list */
      for (int i = 0; i < receive_ids.size(); i++)
      {
        if (can_id == receive_ids[i])
        {
          receive_ids.erase(receive_ids.begin() + i);
          break;
        }
      }
    }
    double t = get_time() - t0;
    /* return if all can_ids have been received or timeout has occured */
    if ((n_can_ids > 0) && receive_ids.empty())
      break;
    if ((timeout >= 0) && (t > timeout))
      break;
    /* update timeout */
    if (timeout >= 0)
      m_can_channel->set_timeout(timeout - t);
  }
  return receive_ids.empty();
}
