#include <string.h>
#include <stdio.h>
#include <zmq.h>
#include "mex.h"

#define BUFLEN 256
char* command;
char zmq_channel[BUFLEN];
void *ctx, *socket;
int result, rc;
static int initialized = 0;

void cleanup( void ){
  mexPrintf("ZMQMEX: closing sockets and context.\n");
  zmq_close( socket );
  zmq_term( ctx );
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  if (!initialized) {
    mexPrintf("ZMQMEX: creating a 2 thread ZMQ context.\n");
    ctx = zmq_init(2);
    initialized = 1;
    mexAtExit(cleanup);
  }

  if ( mxIsChar(prhs[0]) != 1)
    mexErrMsgTxt("Could not read string. (1st argument)");
  command = mxArrayToString(prhs[0]);
//  printf( "Got: %s\n", command );

  if( strcmp(command, "publish")==0 ){
    // Initialize the Context
    socket = zmq_socket (ctx, ZMQ_PUB);
    if (nrhs != 2)
      mexErrMsgTxt("Please provide a name for the ZMQ channel");
    if ( mxIsChar(prhs[1]) != 1)
      mexErrMsgTxt("Could not read string. (2nd argument)");
    sprintf(zmq_channel, "ipc:///tmp/%s", mxArrayToString(prhs[1]) );
    printf("Subscribing to %s.\n",zmq_channel);
    rc = zmq_bind( socket, zmq_channel );
  } else if (strcasecmp(command, "send") == 0){
    // Send stuff on a channel
    if(!socket)
      mexErrMsgTxt("Please initialize your context");
    if (nrhs != 2)
      mexErrMsgTxt("Please provide a string message to send");
    if ( mxIsChar(prhs[1]) != 1)
      mexErrMsgTxt("Could not read string. (2nd argument)");
    size_t msglen = (mxGetM(prhs[1]) * mxGetN(prhs[1])) + 1;
    char* msg = mxArrayToString(prhs[1]);
    int nbytes = zmq_send( socket, (void*)msg, msglen, 0 );
    if(nbytes!=msglen)
      mexErrMsgTxt("Did not send correct number of bytes.");
//    printf("Sent %d bytes: %s\n",nbytes,msg);
  }

}
