-- Pretty much exactly this:
-- http://luajit.org/ext_ffi_tutorial.html
local ok, ffi = pcall(require, 'ffi')
if not ok then return require'zlib' end

ffi.cdef[[
unsigned long compressBound(unsigned long sourceLen);
int compress2(uint8_t *dest, unsigned long *destLen,
	      const uint8_t *source, unsigned long sourceLen, int level);
int uncompress(uint8_t *dest, unsigned long *destLen,
	       const uint8_t *source, unsigned long sourceLen);
]]
local z = ffi.load'z'

local zlib = {}

local nBound = 8
local n, sz, res
local buf = ffi.new("uint8_t[?]", nBound)
local buflen = ffi.new("unsigned long[1]", nBound)

function zlib.compress(txt, len)
  sz = len or #txt
  n = z.compressBound(sz)
  if n > nBound then
    nBound = n
    buf = ffi.new("uint8_t[?]", nBound)
    --buflen = ffi.new("unsigned long[1]", n)
    buflen[0] = n
  end
  -- Just make fast, so use 1 for level
  res = z.compress2(buf, buflen, txt, sz, 9)
  if res~=0 then return end
  -- Reset the bounds
  n = buflen[0]
  buflen[0] = nBound
  -- Return a string
  return ffi.string(buf, n)
end


function zlib.uncompress(comp, n)
	n = n or 1024
  local buf = ffi.new("uint8_t[?]", n)
  local buflen = ffi.new("unsigned long[1]", n)
  local res = z.uncompress(buf, buflen, comp, #comp)
  if res ~= 0 then return end
  return ffi.string(buf, buflen[0])
end

return zlib
