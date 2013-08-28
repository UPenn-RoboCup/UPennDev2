/*
x = Comm;

MEX file to send and receive UDP messages.
Daniel D. Lee, 6/09 <ddlee@seas.upenn.edu>
*/

#include "mex.h"

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <strings.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define MAX_NUM_FD 10

#define PORT 54321
#define MAX_LENGTH 65536 /* 16000 */
int socket_cnt = 0;
int send_fd[MAX_NUM_FD];
mwSize ret_sz[]={1};

void mexExit(void)
{
	mexPrintf("UDP_send: closing sockets and context.\n");
	for(int i=0;i<socket_cnt;i++){
		int fd = send_fd[i];
		if ( send_fd[i] > 0 ){
			close( send_fd[i] );
		}
	}
}

//Now usage: 
// udp_send('init', IP, PORT)
// udp_send('send', PORT, DATA)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	static int init = 0;
	if(init==0){
		mexAtExit(mexExit);
		init = 1;
	}
  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
		mexErrMsgTxt("Need to input string argument");
  
	/* Grab the command */
	char *fname = mxArrayToString(prhs[0]);
	
  if ( strcmp(fname, "init" ) == 0) {
  	if( socket_cnt >= MAX_NUM_FD )mexErrMsgTxt("Not enough sockets available!");
    char *ip_str = mxArrayToString(prhs[1]);
		double* port_ptr = (double*)mxGetData(prhs[2]);
		int port = (int)port_ptr[0];
		struct hostent *hostptr = gethostbyname( ip_str );
		if (hostptr == NULL)mexErrMsgTxt("Could not get hostname");
		int fd = socket(AF_INET, SOCK_DGRAM, 0);
		if (fd < 0)	mexErrMsgTxt("Could not open datagram send socket");
		int i = 1;
		if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST,
		(const char *) &i, sizeof(i)) < 0){
			close( fd );
			mexErrMsgTxt("Could not set broadcast option");
		}
		i = 1;
		if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR,
		(const char *) &i, sizeof(i)) < 0){
			close( fd );
			mexErrMsgTxt("Could not set reuse option");
		}
		struct sockaddr_in dest_addr;
		bzero((char *) &dest_addr, sizeof(dest_addr));
		dest_addr.sin_family = AF_INET;
		bcopy(hostptr->h_addr, (char *) &dest_addr.sin_addr, hostptr->h_length);
		dest_addr.sin_port = htons( port );
		if ( connect(fd, (struct sockaddr *) &dest_addr,sizeof(dest_addr)) < 0){
			close( fd );
			mexErrMsgTxt("Could not connect to destination address");
		}
		mexPrintf("Broadcasting udp_send on %s:%d\n", ip_str, port);
		send_fd[socket_cnt] = fd;
		ret_sz[0] = 1;
		plhs[0] = mxCreateNumericArray(1,ret_sz,mxUINT8_CLASS,mxREAL);
		uint8_t* out = (uint8_t*)mxGetData(plhs[0]);
		out[0] = socket_cnt;
		socket_cnt++;
	} else {
		/* Get the socket id */
		uint8_t* socketid = (uint8_t*)mxGetData(prhs[1]);
		int socket = socketid[0];
		if( socket > socket_cnt )
			mexErrMsgTxt("Bad socket id!");
		int fd = send_fd[socket];
		int nBytes = mxGetNumberOfElements(prhs[2])*mxGetElementSize(prhs[2]);
		if( nBytes > MAX_LENGTH ) {
			mexErrMsgTxt("Trying to send too many bytes");
		}
		int ret = send( fd, mxGetData(prhs[2]), nBytes, 0 );
		plhs[0] = mxCreateDoubleScalar(ret);
	}
}
