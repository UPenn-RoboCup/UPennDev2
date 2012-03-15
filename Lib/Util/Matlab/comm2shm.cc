
#include "libMonitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <iostream>
#include <deque>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <bitset>
#include <regex>

using namespace boost::interprocess;

//typedef double value_t;
typedef uint64_t value_t;

std::string **dataYUV;
uint64_t *px;

bool firstFrame = true;
bool **sectionFull; 

const size_t msgHeader = 9;

void fnExit(void) { 
	close_comm(); 
	if (firstFrame) {
		for (size_t i = 0; i < 3; i++)
			delete [] dataYUV[i];
		delete [] dataYUV;
	}
	delete [] px;
}

inline void bufferClean(size_t division) {
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < division; j++)
			sectionFull[i][j] = false;
}

inline bool imageCheck(size_t division) {
	bool ret = true;
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < division; j++) {
			ret = ret && sectionFull[i][j];
	}
	return ret;
}

int pushYUYV(size_t division) {
	const char *name = "netImage";
	size_t shmSize = dataYUV[0][0].size()* division * 8+ 65536; 
//	std::cout << shmSize << std::endl;
	managed_shared_memory *shm;

	shm = new managed_shared_memory(open_only,name);

	const char *key = "yuyv";
	std::pair<value_t*, std::size_t> ret;
	try {
		ret = shm->find<value_t>(key);
	}
	catch (std::exception & e) {
		std::cout << e.what() << std::endl;
	}
	value_t *pr = ret.first;
	int n = ret.second;

	size_t keySize = (dataYUV[0][0].size()-msgHeader)/2;	
//	std::cout << n << std::endl;
	if (pr == NULL) {
//		std::cout << "NULL" << std::endl;
		try {
			pr = shm->construct<value_t>(key)[(keySize)*division]();
		}
		catch (bad_alloc &ex) {
			std::cerr << ex.what() << std::endl;
		}
	} 
//	std::cout << "Found key in shared memory" << std::endl;
	else if (n != (keySize)) {
		try {
			shm->destroy_ptr(pr);
			pr = shm->construct<value_t>(key)[(keySize)*division]();
		}
		catch (bad_alloc &ex) {
			std::cerr << ex.what() << std::endl;
		}
	}

	size_t idxShm = 0;
	for (size_t i = 0; i < division; i++)
		for (size_t j = msgHeader; j < dataYUV[0][0].size(); j+=2) {
		uint64_t high = ((uint64_t)dataYUV[0][i][j]<<26)   | ((uint64_t)dataYUV[2][i][j]<<18)   
										| ((uint64_t)dataYUV[0][i][j]<<10)   | ((uint64_t)dataYUV[1][i][j]<<2);
		uint64_t low  = ((uint64_t)dataYUV[0][i][j+1]<<26) | ((uint64_t)dataYUV[2][i][j+1]<<18) 
										| ((uint64_t)dataYUV[0][i][j+1]<<10) | ((uint64_t)dataYUV[1][i][j+1]<<2);	
		pr[idxShm++]  = (low << 32) | high;
	}
	
	return 0;
}

int restoreYUYV(const char *data, size_t size) {
	size_t type = data[0]-18;
	size_t division = data[7];
	size_t section = data[8]-1;
	size_t width = data[1] * 128 + data[2];
	size_t height = data[3] * 128 + data[4];
	if (firstFrame) {
		dataYUV = new std::string *[3];
		sectionFull = new bool *[3];
		for (size_t i = 0; i < 3; i++) {
			dataYUV[i] = new std::string [division];
			sectionFull[i] = new bool [division];
			for (size_t k = 0; k < division; k++)
				sectionFull[i][k] = false;
		}
		px = new uint64_t [width*height/2];
		firstFrame = false;
	}
	std::string *Que = &dataYUV[type][section];
	std::string Data(data);
	try {
		Que->swap(Data);
	}
	catch (std::exception &e) {
		std::cout << e.what() << std::endl;
	}
	sectionFull[type][section] = true;
	
	bool ret = imageCheck(division);
	if (ret) {
		std::cout << "Image Full" << std::endl;
		pushYUYV(division);
		bufferClean(division);
	}
	return 1;
}

int parseMsgs(const char * msg, size_t size)
{
	size_t type = msg[0];
	std::cout << msg << std::endl;
	std::string str(msg);
//	std::smatch result;
//	std::regex rx("\{\[+?");
//	std::cout << std::regex_search(str.c_str(),result,rx,0) << std::endl;
	switch (type)
	{
		case 11:
			break;
		case 16:
			break;
		case 17:
			break;
		case 18:
		case 19:
		case 20:
		//	restoreYUYV(msg, size);
			break;
	}
	return 1;
}

int main(int argc, char ** argv)
{
	atexit(fnExit);

	const char *name = "netImage";
	size_t division = 4;
	size_t shmSize = 320*480 * 8+ 65536; 
	managed_shared_memory *shm;
	shared_memory_object::remove(name);
	shm = new managed_shared_memory(create_only,name,shmSize);
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
			parseMsgs((const char *)get_front_data(),n);
    	pop_data();
	  }
	} 
}
