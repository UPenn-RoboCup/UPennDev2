/*
	 to open TCP connection and return Matlab file identifier.

	 Daniel D. Lee, 12/01.
	 <ddlee@ddlee.com>
	 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ctype.h>

#include <lua.hpp>

/*
	 fidFdopen takes a Unix file descriptor fd and
	 returns a Matlab-usable file identifier with
	 permissions given by mode.
	 */

#define MT_NAME "tcp_mt"

typedef struct {
  int sock_fd;
  const char * ip;
  int port;
  int nonblock;
} structTCP;

static structTCP * lua_checktcp(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid Udp udata");
  return (structTCP *)ud;
}

static int lua_tcp_index(lua_State *L) {
  //  lua_checktcp(L, 1);
  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

//int fiFdopen(int fd, char *mode, char *format)
//{
//	int fopenfd, matlab_fid;
//	int fopen_nlhs, fopen_nrhs;
//	mxArray *fopen_lhs[1], *fopen_rhs[3];
//	struct stat stat_tmpfile, statbuf;
//	char *tmpfilename;
//
//	/* First open a temporary file and get file status info */
//
//	if ((tmpfilename = tmpnam(NULL)) == NULL)
//		mexErrMsgTxt("Could not generate tmpfile name.");
//
//	if ((fopenfd = open(tmpfilename, O_CREAT|O_RDWR, 0777)) < 0)
//		mexErrMsgTxt("Could not create tmpfile");
//	if (fstat(fopenfd, &stat_tmpfile) < 0)
//		mexErrMsgTxt("Could not stat tmpfile");
//
//	/* Now close the temporary file */
//
//	close(fopenfd);
//
//	/* Now call the Matlab function "fopen" to open the same temporary file */
//
//	fopen_nlhs = 1;
//	fopen_nrhs = 3;
//	fopen_rhs[0] = mxCreateString(tmpfilename);
//	fopen_rhs[1] = mxCreateString(mode);
//	fopen_rhs[2] = mxCreateString(format);
//	mexCallMATLAB(fopen_nlhs, fopen_lhs, fopen_nrhs, fopen_rhs, "fopen");
//
//	if ((matlab_fid = mxGetScalar(fopen_lhs[0])) < 0)
//		mexErrMsgTxt("Could not open tmpfile");
//
//	/* Now delete temporary file and remap Matlab fopen to new file descriptor */
//
//	if (unlink(tmpfilename) < 0)
//		mexErrMsgTxt("Could not unlink tmpfile");
//
//	if (fstat(fopenfd, &statbuf) < 0)
//		mexErrMsgTxt("Could not stat file descriptor");
//	if ((statbuf.st_dev != stat_tmpfile.st_dev) ||
//			(statbuf.st_ino != stat_tmpfile.st_ino))
//		mexErrMsgTxt("Could not find matching file descriptor");
//
//	if (dup2(fd, fopenfd) < 0)
//		mexErrMsgTxt("Could not duplicate file descriptor.");
//
//	close(fd);
//
//	return matlab_fid;
//}

static int lua_tcp_open(lua_State *L) {
  structTCP *p = (structTCP *)lua_newuserdata(L, sizeof(structTCP)); 

	int sock_fd;
	int i, len, source_addr_len, dims[2];
	int buflen, port = 80, nonblock = 0;
	struct sockaddr_in local_addr, serv_addr;
	struct hostent *hostptr;
	char *buf;

  // input host address
  p->ip = luaL_checkstring(L, 1);
	if ((hostptr = gethostbyname(p->ip)) == NULL)
		return luaL_error(L, "Could not get hostname.");
  
  // input host port 
  p->port = luaL_checkint(L, 2);

  // input nonblock mode or not
  p->nonblock = luaL_optint(L, 3, 0);

	/* Get host info */
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy(hostptr->h_addr, (char *) &serv_addr.sin_addr, hostptr->h_length);
	serv_addr.sin_port = htons(p->port);

	if ((p->sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		return luaL_error(L, "Could not open socket.");

	/* Set read buffer for performance in MacOS X */
	i = 65535;
	if (setsockopt(p->sock_fd, SOL_SOCKET, SO_RCVBUF, &i, sizeof(int)) < 0)
		return luaL_error(L, "Could not set socket option.");

	if (connect(p->sock_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		return luaL_error(L, "Could not connect to server.");

	/* Set nonblocking I/O */
	if (nonblock) {
		if (fcntl(p->sock_fd, F_SETFL, O_NONBLOCK) == -1) {
			close(p->sock_fd);
			return luaL_error(L, "Could not set nonblocking I/O.");
		}
	}

  lua_pushinteger(L, p->sock_fd);

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	return 1;

//	/* Create output arguments */
//	plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
//	/* mxGetPr(plhs[0])[0] = fiFdopen(sock_fd, "w+", "native"); */
//	mxGetPr(plhs[0])[0] = fiFdopen(sock_fd, "a+", "ieee-le");
//  return 1;
}

static int lua_tcp_close(lua_State *L) {
  structTCP *p = lua_checktcp(L, 1);

  close(p->sock_fd);
//  if (p->recv_fd > 0 && p->init_recv) {
//    close(p->recv_fd);
//    p->init_recv = false;
//    p->recv_queue->clear();
//    delete p->recv_queue;
//    p->raw_recv_queue->clear();
//    delete p->raw_recv_queue;
//    lua_pushboolean(L, 1);
//    return 1;
//  }
//
//  if (p->send_fd > 0 && p->init_send) {
//    close(p->send_fd);
//    p->init_send = false;
//    lua_pushboolean(L, 1);
//    return 1;
//  }
//
//	lua_pushboolean(L,0);
	return 1;
}

static const struct luaL_Reg tcp_function [] = {
	{"open", lua_tcp_open},
	{NULL, NULL}
};

static const luaL_Reg tcp_methods[] = {
//  {"close", lua_tcp_close},
//  {"__tostring", lua_tcp_string},
  {"__gc", lua_tcp_close},
  {"__index", lua_tcp_index},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_tcp(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
	luaL_setfuncs( L, tcp_methods, 0 );
	luaL_newlib(L, tcp_function);
#else
  luaL_register(L, NULL, tcp_methods);
	luaL_register(L, "tcp", tcp_function);
#endif  
	return 1;
}
