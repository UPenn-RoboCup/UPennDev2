local carray = require 'carray'
--local ffi = require 'ffi'

dd = carray.double(5)
dd[2] = 4
dd[5] = 342
for i = 1, 5 do
  print(dd[i])
end
carray.cast()

--cdata = ffi.cast('double*', dd:pointer())
--print(cdata[1])
--[[
local torch = require'torch'
local tmp   = torch.FloatTensor(5):zero()
local tt    = carray.float(5)
tt[1] = 5
tt[2] = 64
tt[3] = 3.14
local tbl = tt:table()
print(tbl[1])
tt:tensor( tmp, 3, 2 )
print('Tensor[1:5]:',tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])
--]]
