-- Pretty much exactly this:
-- http://luajit.org/ext_ffi_tutorial.html

ffi.cdef[[
unsigned long compressBound(unsigned long sourceLen);
int compress2(uint8_t *dest, unsigned long *destLen,
	      const uint8_t *source, unsigned long sourceLen, int level);
int uncompress(uint8_t *dest, unsigned long *destLen,
	       const uint8_t *source, unsigned long sourceLen);
]]
local z = ffi.load'z'

local zlib = {}

local nBound = 512
local buf = ffi.new("uint8_t[?]", nBound)
local buflen = ffi.new("unsigned long[1]", n)

function zlib.compress(txt)
  local n = z.compressBound(#txt)
  if n>nBound then
    nBound = n
    buf = ffi.new("uint8_t[?]", nBound)
    buflen = ffi.new("unsigned long[1]", n)
  end
  -- Just make fast, so use 1 for level
  local res = z.compress2(buf, buflen, txt, #txt, 1)
  assert(res == 0)
  return ffi.string(buf, buflen[0])
end

function zlib.compress_cdata (ptr, len)
  local n = z.compressBound(len)
  if n>nBound then
    nBound = n
    buf = ffi.new("uint8_t[?]", nBound)
    buflen = ffi.new("unsigned long[1]", n)
  end
  -- Just make fast, so use 1 for level
  local res = z.compress2(buf, buflen, ffi.cast('uint8_t*',ptr), len, 1)
  assert(res == 0)
  return ffi.string(buf, buflen[0])
end

return zlib