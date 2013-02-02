--result of cpp stdio.h from mingw (FILE made opaque)
local ffi = require'ffi'

--[[
{
	char* _ptr;
	int _cnt;
	char* _base;
	int _flag;
	int _file;
	int _charbuf;
	int _bufsiz;
	char* _tmpfname;
}
]]
ffi.cdef[[
typedef struct _iobuf FILE;
typedef long long fpos_t;
]]
