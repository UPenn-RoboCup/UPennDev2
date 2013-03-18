local ffi = require'ffi'
require'stdio_h'
local M = {C = ffi.C}

function M.readfile(file, format)
	local f = ffi.C.fopen(file, format=='t' and 'r' or 'rb')
	ffi.C.fseek(f, 0, ffi.C.SEEK_END)
	local sz = ffi.C.ftell(f)
	ffi.C.fseek(f, 0, ffi.C.SEEK_SET)
	local buf = ffi.new('uint8_t[?]', sz)
	ffi.C.fread(buf, 1, sz, f)
	ffi.C.fclose(f)
	return buf, sz
end

function M.writefile(file, data, sz, format)
	local f = ffi.C.fopen(file, format=='t' and 'w' or 'wb')
	ffi.C.fwrite(data, 1, sz, f)
	ffi.C.fclose(f)
end

return M
