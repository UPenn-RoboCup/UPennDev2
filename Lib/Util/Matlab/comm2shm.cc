
#include "libMonitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <iostream>

void fnExit(void) { close_comm(); }

int main(int argc, char ** argv)
{
	atexit(fnExit);

//  if ((argc < 2)) {
//    printf("Incorrect input argument\n");
//	}

	std::string ip("192.168.123.255");

  static bool init = false;
  if (!init) {
    printf("Initializing monitorComm...\n");
    int ret = init_comm(ip.c_str());
		std::cout << ip << std::endl;
    switch( ret ){
      case 0:
        printf("Success!\n");
        break;
      case -1:
        printf("Could not connect to destination address");
        break;
      case -2:
        printf("Could not open datagram recv socket");
        break;
      case -3:
        printf("Could not bind to port");
        break;
      case -4:
        printf("Could not set nonblocking mode");
        break;
      case -5:
        printf("Could not get hostname");
        break;
      case -6:
        printf("Could not open datagram send socket");
        break;
      case -7:
        printf("Could not set broadcast option");
        break;
      default:
        break;
    }
   	
		if (ret!=0) { 
			printf("exit\n");	
			exit (1);
		}
    init = true;
  }

  // Process incoming messages:
	while (true) {
	  process_message();

		double Qsize = getQueueSize();
		while (!queueIsEmpty()) {
   		int n = get_front_size();
    	std::string packet((const char *)get_front_data());
			std::cout << (int)packet[0] << std::endl;
    	pop_data();
	  }
	} 
}
