#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>


int main(int argc,char **argv) {

  int z; // Status return code
  struct ifreq ifr;
  struct can_frame frame;
  struct sockaddr_can addr;
  int bytes_sent, bytes_read;
  
  // Create a pair of local 
  z = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  
  if ( z == -1 ) {
    fprintf(stderr, "%s\n", strerror(errno));
    return 1; // Failed
  }
  
  strcpy(ifr.ifr_name, "can0");
  ioctl(z, SIOCGIFINDEX, &ifr); // ifr.ifr_ifindex gets filled 
                                // with that device's index
 
  // Select that CAN interface, and bind the socket to it.
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind( z, (struct sockaddr*)&addr, sizeof(addr) );

  // Read a message back from the CAN bus
  printf("Reading...\n");
  bytes_read = read( z, &frame, sizeof(frame) );
  
  printf("Received %i bytes\n", bytes_read);
  printf("Data: %s\n", frame.data);


  return 0;
}
