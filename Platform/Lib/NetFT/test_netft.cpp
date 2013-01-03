#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <vector>
#include "co_master.h"
#include "netft_node.h"

#define NODE_ID 8
#define TIMEOUT 1

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

int main()
{
  /* initialize can bus */
  can_channel channel;
  if (channel.open_channel("can2") < 0)
    fprintf(stderr, "Error: unable to open can interface\n");

  /* initialize net-ft */
  netft_node node(NODE_ID);
  node.set_force_scale_factor(1);
  node.set_torque_scale_factor(1);
  std::vector<struct can_frame> &ft_buffers = node.get_can_buffers();

  /* initialize master */
  co_master master(&channel); 
  master.enable_debug_print(true);
  for (int i = 0; i < ft_buffers.size(); i++)
  {
    master.register_can_buffer(&ft_buffers[i]);
  }

  /* start polling the sensor */ 
  co_can_id receive_ids[] = {NODE_ID + 5, NODE_ID + 6};
  while (1)
  {
    double forces[3];
    double torques[3];
    /* request short data */
    struct can_frame frame = node.get_short_request();
    master.send_can_frame(&frame);
    assert(master.receive(receive_ids, 2, TIMEOUT));
    /* get force torque values */
    node.update_data();
    node.get_forces(forces);
    node.get_torques(torques);
    /* display results */
    fprintf(stderr, "forces : %f, %f, %f\n", forces[0], forces[1], forces[2]);
    fprintf(stderr, "torques : %f, %f, %f\n", torques[0], torques[1], torques[2]);
    fprintf(stderr, "status : %d\nsample : %d\n", node.get_status(), node.get_sample());
    usleep(1000);
    system("clear");
  }
 
  channel.close_channel();
  return 0;
}
