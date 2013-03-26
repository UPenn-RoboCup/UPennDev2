#include <string>
#include <stdio.h>
#include <zmq.h>
#include <unistd.h>
#include "mex.h"
#define BUFLEN 1024
#define MAX_SOCKETS 10

char* command;
char zmq_channel[200];//name
void *ctx;
void * sockets[MAX_SOCKETS];
uint8_t socket_cnt = 0;
int result, rc;
static int initialized = 0;
const mwSize ret_sz[]={1};
char recv_buffer[BUFLEN];

void cleanup( void ){
  mexPrintf("ZMQMEX: closing sockets and context.\n");
  for(int i=0;i<socket_cnt;i++)
    zmq_close( sockets[i] );
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
    if (nrhs != 2)
      mexErrMsgTxt("Please provide a name for the ZMQ channel");
    if ( mxIsChar(prhs[1]) != 1)
      mexErrMsgTxt("Could not read string. (2nd argument)");
    char* ch_name = mxArrayToString(prhs[1]);
    sprintf(zmq_channel, "ipc:///tmp/%s", ch_name );
    printf("Subscribing to %s.\n",zmq_channel);
    sockets[socket_cnt] = zmq_socket (ctx, ZMQ_PUB);
    rc = zmq_bind( sockets[socket_cnt], zmq_channel );
    printf("Creating Socket: %d, %u\n",socket_cnt,sockets[socket_cnt]);
    plhs[0] = mxCreateNumericArray(1,ret_sz,mxUINT8_CLASS,mxREAL);
    uint8_t* out = (uint8_t*)mxGetData(plhs[0]);
    out[0] = socket_cnt;
    socket_cnt++;
  } else if( strcmp(command, "subscribe")==0 ){
    // Initialize the Context
    if (nrhs != 2)
      mexErrMsgTxt("Please provide a name for the ZMQ channel");
    if ( mxIsChar(prhs[1]) != 1)
      mexErrMsgTxt("Could not read string. (2nd argument)");
    char* ch_name = mxArrayToString(prhs[1]);
    sprintf(zmq_channel, "ipc:///tmp/%s", ch_name );
    printf("Subscribing to %s.\n",zmq_channel);
    sockets[socket_cnt] = zmq_socket (ctx, ZMQ_SUB);
    zmq_setsockopt( sockets[socket_cnt], ZMQ_SUBSCRIBE, "", 0 );
    rc = zmq_connect( sockets[socket_cnt], zmq_channel );
    printf("Creating Socket: %d, %u\n",socket_cnt,sockets[socket_cnt]);
    plhs[0] = mxCreateNumericArray(1,ret_sz,mxUINT8_CLASS,mxREAL);
    uint8_t* out = (uint8_t*)mxGetData(plhs[0]);
    out[0] = socket_cnt;
    socket_cnt++;
  } else if (strcasecmp(command, "send") == 0){
    // Send stuff on a channel
    if (nrhs != 3)
      mexErrMsgTxt("Please provide a socket id and a string message to send");
    if ( !mxIsClass(prhs[1],"uint8") || mxGetNumberOfElements( prhs[1] )!=1 )
      mexErrMsgTxt("Please provide a valid handle");
    //size_t socket_id_len = mxGetNumberOfElements( prhs[1] );
    uint8_t* socketid = (uint8_t*)mxGetData(prhs[1]);
    int socket = socketid[0];
    printf("Accessing socket %u, %u\n",socket, sockets[socket]);
    if( socket>socket_cnt)
      mexErrMsgTxt("Bad socket id!");
    if ( !mxIsChar(prhs[2]) )
      mexErrMsgTxt("Could not read string. (3rd argument)");
    size_t msglen = (mxGetM(prhs[2]) * mxGetN(prhs[2])) + 1;
    char* msg = mxArrayToString(prhs[2]);
    int nbytes = zmq_send( sockets[ socket ], (void*)msg, msglen, 0 );
    printf("Sent %d bytes: %s\n",nbytes,msg);
    if(nbytes!=msglen)
      mexErrMsgTxt("Did not send correct number of bytes.");
  } else if (strcasecmp(command, "receive") == 0){
    if (nrhs != 2)
      mexErrMsgTxt("Please provide a socket id.");
    if ( !mxIsClass(prhs[1],"uint8") || mxGetNumberOfElements( prhs[1] )!=1 )
      mexErrMsgTxt("Please provide a valid handle");
    uint8_t* socketid = (uint8_t*)mxGetData(prhs[1]);
    int socket = socketid[0];
    printf("Accessing socket %u, %u\n",socket, sockets[socket]);
    if( socket>socket_cnt)
      mexErrMsgTxt("Bad socket id!");
    //int nbytes = zmq_recv(sockets[socket], recv_buffer, BUFLEN, ZMQ_DONTWAIT);
    int nbytes = zmq_recv(sockets[socket], recv_buffer, BUFLEN, 0);
    printf("Received %d bytes\n",nbytes);
  } else
    mexErrMsgTxt("Unrecognized command");

}
