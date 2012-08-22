#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>

#include <time.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

using namespace std;

int main(int argc,char **argv) {

  int z; // Status return code
  struct ifreq ifr;
  struct can_frame frame;
  struct sockaddr_can addr;
  int bytes_sent, bytes_read, total_wrote, total_read;
  int failures;
  char payload[] = "payload\0";
  int i;
  timeval tv1, tv2;
  double elapsedTime;
  
  time_t  t0, t1;
  clock_t c0, c1;
  
  // Create a pair of local 
  z = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  
  if ( z == -1 ) {
    fprintf(stderr, "%s\n", strerror(errno));
    return 1; // Failed
  }
  
  strcpy(ifr.ifr_name, "can1");
  ioctl(z, SIOCGIFINDEX, &ifr); // ifr.ifr_ifindex gets filled 
                                // with that device's index
 
  // Select that CAN interface, and bind the socket to it.
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind( z, (struct sockaddr*)&addr, sizeof(addr) );
  
  // Send a message to the CAN bus
  frame.can_id = 0x123;
  strcpy((char*)frame.data, payload);
  frame.can_dlc = strlen( (char*)frame.data ) +1;
  printf("Writing...\n");
  
  t0 = time(NULL);
  c0 = clock();   // start timer here...
  gettimeofday(&tv1, NULL);
  
  total_wrote = total_read = failures = 0;
  for(i=0; i<10000; i++)
  {
  
    bytes_sent = write( z, &frame, sizeof(frame) );
    while(bytes_sent != sizeof(frame))
    {
      failures++;
      bytes_sent = write( z, &frame, sizeof(frame)); // keep trying till we get it
    }
    
    total_wrote += bytes_sent;
    
    // Send a message to the CAN bus
    frame.can_id = 0x123;
    strcpy((char*)frame.data, payload);
    frame.can_dlc = strlen( (char*)frame.data ) + 1;
    
    //if(i == 0)
    {
      // Read a message back from the CAN bus
      total_read += bytes_read = read( z, &frame, sizeof(frame) );
      
      //gettimeofday(&tv2, NULL);
      //cout<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;
      //printf("\tlatency:      %li usec\n", (tv2.tv_usec - tv1.tv_usec));
    }
  }
  
  t1 = time(NULL);
  c1 = clock();
  
  gettimeofday(&tv2, NULL);
  printf("\tTotal:      %li usec\n", (tv2.tv_usec - tv1.tv_usec));
  
  printf("\ttotal read:     %i\n", total_read);
  printf("\ttotal written:  %i\n", total_wrote);
  printf("\twrite failures: %i\n", failures);
  printf("\telapsed CPU time:        %f seconds\n", (float) (c1 - c0)/CLOCKS_PER_SEC);
  
  printf("Received %i bytes\n", bytes_read);
  printf("Data: %s\n", frame.data);

  return 0;
}
