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

  int nframes = 1000;
  int can0, can1; // Status return code
  struct ifreq ifr0, ifr1;
  struct can_frame frame;
  struct sockaddr_can addr0, addr1;
  int bytes_sent, bytes_read, total_wrote, total_read;
  int failures;
  char payload[] = "payload\0";
  int i;
  timeval tv1, tv2;
  double elapsedTime;
  
  time_t  t0, t1;
  clock_t c0, c1;
  
  // Create a pair of local sockets
  can0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  can1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  
  if ( can0 == -1 || can1 == -1) {
    fprintf(stderr, "%s\n", strerror(errno));
    return 1; // Failed
  }
  
  strcpy(ifr0.ifr_name, "can0");
  strcpy(ifr1.ifr_name, "can1");

  ioctl(can0, SIOCGIFINDEX, &ifr0); // ifr.ifr_ifindex gets filled 
  ioctl(can1, SIOCGIFINDEX, &ifr1); // ifr.ifr_ifindex gets filled 
 
  // Select CAN interface, and bind the socket to it.
  addr0.can_family = AF_CAN;
  addr0.can_ifindex = ifr0.ifr_ifindex;
  addr1.can_family = AF_CAN;
  addr1.can_ifindex = ifr1.ifr_ifindex;
  bind( can0, (struct sockaddr*)&addr0, sizeof(addr0) );
  bind( can1, (struct sockaddr*)&addr1, sizeof(addr1) );
  
  t0 = time(NULL);
  c0 = clock();   // start timer here...
  gettimeofday(&tv1, NULL);
  
  printf("Writing...\n");

  total_wrote = total_read = failures = 0;
  for(i=0; i<nframes; i++)
  {
  
    // Send a message to CAN0 
    frame.can_id = 0x123;
    strcpy((char*)frame.data, payload);
    frame.can_dlc = strlen( (char*)frame.data ) + 1;

    bytes_sent = write( can0, &frame, sizeof(frame) );
    while(bytes_sent != sizeof(frame))
    {
      failures++;
      bytes_sent = write( can0, &frame, sizeof(frame)); // keep trying till we get it
    }
    total_wrote += bytes_sent;

    // Read the message on CAN1
    bytes_read = read( can1, &frame, sizeof(frame) );
    while(bytes_read != bytes_sent)
    {
      bytes_read = read( can1, &frame, sizeof(frame) );
    }
    total_read += bytes_read;
    
  }
  
  t1 = time(NULL);
  c1 = clock();
  
  gettimeofday(&tv2, NULL);
  elapsedTime = (tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec)*1e-6;
  
  printf("\tTotal:      %f sec\n", elapsedTime);
  printf("\tTotal:      %f fps\n", nframes/elapsedTime);
  
  printf("\ttotal read:     %i\n", total_read);
  printf("\ttotal written:  %i\n", total_wrote);
  printf("\twrite failures: %i\n", failures);
  printf("\telapsed CPU time:        %f seconds\n", (float) (c1 - c0)/CLOCKS_PER_SEC);
  
  printf("Received %i bytes\n", bytes_read);
  printf("Data: %s\n", frame.data);

  return 0;
}
