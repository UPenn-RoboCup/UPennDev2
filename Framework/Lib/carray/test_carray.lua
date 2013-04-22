local carray = require 'carray'

dd = carray.double(5)
dd[2] = 4
dd[5] = 342
for i = 1, 5 do
  print(dd[i])
end
carray.cast()

--[[
print("LuaJIT testing...")
local ffi = require 'ffi'
cdata = ffi.cast('double*', dd:pointer())
print(cdata[1])
assert( cdata[1]==dd[2] )
--]]
