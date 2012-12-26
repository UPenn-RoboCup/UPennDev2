/*
 * netft_node : CAN communication interface for ATI net-ft boards 
 * author : Mike Hopkins
 */

#include "netft_node.h"
#include <stdint.h>
#include <string.h>

netft_node::netft_node(co_can_id node_id)
{
  set_node_id(node_id);
  memset(m_forces, 0, 3*sizeof(int));
  memset(m_torques, 0, 3*sizeof(int));
  m_status = 0;
  m_sample = 0;
  m_force_scale_factor = 1;
  m_torque_scale_factor = 1;
}

co_can_id netft_node::get_node_id()
{
  return m_node_id;
}

void netft_node::set_node_id(co_can_id node_id)
{
  m_node_id = node_id;
  m_can_buffers.resize(6);
  for (int i = 0; i < 6; i++)
  {
    struct can_frame frame = {node_id + i + 1, 0, 0};
    m_can_buffers[i] = frame;
  }
}

std::vector<struct can_frame> &netft_node::get_can_buffers()
{
  /* return reference to can buffers */
  return m_can_buffers;
}

struct can_frame netft_node::get_long_request()
{
  /* return long request frame */
  struct can_frame frame = {m_node_id, 1, 0x01};
  return frame; 
}

struct can_frame netft_node::get_short_request()
{
  /* return short request frame */
  struct can_frame frame = {m_node_id, 1, 0x02};
  return frame; 
}

struct can_frame netft_node::get_bias_command()
{
  /* return bias command frame */
  struct can_frame frame = {m_node_id, 1, 0x04};
  return frame; 
}

struct can_frame netft_node::get_clear_threshold_command()
{
  /* return clear threshold command frame */
  struct can_frame frame = {m_node_id, 1, 0x08};
  return frame; 
}

void netft_node::set_force_scale_factor(double factor)
{
  m_force_scale_factor = factor;
}

void netft_node::set_torque_scale_factor(double factor)
{
  m_torque_scale_factor = factor;
}

void netft_node::get_forces(double forces[3])
{
  /* get current force data */
  forces[0] = (double)m_forces[0]*m_force_scale_factor/1e6;
  forces[1] = (double)m_forces[1]*m_force_scale_factor/1e6;
  forces[2] = (double)m_forces[2]*m_force_scale_factor/1e6;
}

void netft_node::get_torques(double torques[3])
{
  /* get current torque data */
  torques[0] = (double)m_torques[0]*m_torque_scale_factor/1e6;
  torques[1] = (double)m_torques[1]*m_torque_scale_factor/1e6;
  torques[2] = (double)m_torques[2]*m_torque_scale_factor/1e6;
}

int netft_node::get_status()
{
  /* get current status number */
  return m_status; 
}

int netft_node::get_sample()
{
  /* get current sample number */
  return m_sample;
}

void netft_node::update_data()
{
  /* update member data if can buffers have been updated */
  for (int i = 0; i < m_can_buffers.size(); i++)
  {
    if (m_can_buffers[i].can_dlc > 0)
    {
      switch (m_can_buffers[i].can_id - m_node_id)
      {
        case 1 : 
          m_forces[0] = *(int32_t *)(m_can_buffers[i].data + 0);
          m_torques[0] = *(int32_t *)(m_can_buffers[i].data + 4);
          break;
        case 2 : 
          m_forces[1] = *(int32_t *)(m_can_buffers[i].data + 0);
          m_torques[1] = *(int32_t *)(m_can_buffers[i].data + 4);
          break;
        case 3 : 
          m_forces[2] = *(int32_t *)(m_can_buffers[i].data + 0);
          m_torques[2] = *(int32_t *)(m_can_buffers[i].data + 4);
          break;
        case 4 : 
          m_status = *(int32_t *)(m_can_buffers[i].data + 0);
          m_sample = *(int32_t *)(m_can_buffers[i].data + 4);
          break;
        case 5 : 
          m_forces[0] = *(int16_t *)(m_can_buffers[i].data + 0);
          m_torques[0] = *(int16_t *)(m_can_buffers[i].data + 2);
          m_forces[1] = *(int16_t *)(m_can_buffers[i].data + 4);
          m_torques[1] = *(int16_t *)(m_can_buffers[i].data + 6);
          break;
        case 6 : 
          m_forces[2] = *(int16_t *)(m_can_buffers[i].data + 0);
          m_torques[2] = *(int16_t *)(m_can_buffers[i].data + 2);
          m_status = *(int16_t *)(m_can_buffers[i].data + 4);
          m_sample = *(int16_t *)(m_can_buffers[i].data + 6);
          break;
        default :
          break;
      }
      m_can_buffers[i].can_dlc = 0;
    }
  }
}
