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
  int bytes_sent, bytes_read, total_read;
  char payload[] = "payload\0";
  int i;
  
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

  total_read = 0;
  for(i=0; i<10000; i++)
  {
    // Read a message back from the CAN bus
    //printf("Reading...\n");
    total_read += bytes_read = read( z, &frame, sizeof(frame) );
    
    //printf("Received %i bytes\n", bytes_read);
    //printf("Data: %s\n", frame.data);

    //if(i == 0)
    {
      // Send a message to the CAN bus
      frame.can_id = 0x123;
      strcpy(frame.data, payload);
      frame.can_dlc = strlen( frame.data ) +1;
      bytes_sent = write( z, &frame, sizeof(frame) );
    }
  }
  
  printf("\ttotal read:     %i\n", total_read);
  
  return 0;
}
