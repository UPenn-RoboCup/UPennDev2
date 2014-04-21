dofile'../include.lua'
ffi=require'ffi'
torch = require'torch'
C = ffi.C
ffi.cdef [[
typedef struct __IO_FILE FILE;
size_t fwrite
(const void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
size_t fread
(void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
]]
local fname = 'nodir_edge'
f_r = io.open(fname..'.raw','r')
local w = 164
local h = 124
local sz = w*h
local buf = ffi.new('double[?]',sz)
local n_read = C.fread(buf,1,sz*ffi.sizeof('double'),f_r)
print(buf,n_read)

-- Skip 2 on each size just to prune the convolution
local THRESH = 500
local bin = torch.ByteTensor(w,h)
local bin_flat = torch.ByteTensor(bin)
bin_flat:resizeAs(
torch.ByteTensor(sz)
)

-- copy it in the silly fashion for now :)
for i=1,sz do
	local num = buf[i-1]
	if num>THRESH then
		bin_flat[i] = 255
	else
		bin_flat[i] = 0
	end
end

-- Save the image as a jpeg to see what's up
local jpeg = require'jpeg'
c_g = jpeg.compressor("gray")
f_j = io.open(fname..'.jpeg','w')
local str = c_g:compress(bin)
f_j:write(str)
f_j:close()
