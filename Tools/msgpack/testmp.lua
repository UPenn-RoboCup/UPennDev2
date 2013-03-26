require 'include'
--require 'common'

local util = require 'util'

--local mp = require'luajit-msgpack-pure'
--local mp = require'luajit-msgpack'
local mp = require'MessagePack'
--local mpc = require 'cmsgpack'

a = {1,2,3,4,5,6,7,8,9}
tbl = {['a'] = 45, ['rt']='4444'}
print('packing 1:')
for i,v in pairs(tbl) do
  print(i,v)
end
tbl1 = {['b'] = 45, ['t']='4444', ['arr'] = a}
print('\npacking 2:')
for i,v in pairs(tbl1) do
  print(i,v)
end

require 'cmsgpack'
local str = mp.pack(tbl1)
--local str = mp.pack('fdfdfd')
--local str = mp.pack(-4324252)
cmsgpack.new(str, #str)

local file = io.open('msg1', 'rb')
str = file:read('*a')
file:close()
print(#str)
obj = mp.unpack(str)
print(obj, #obj)
--util.ptable(tbl)
--print(str:byte(1, #str))

--
str = mp.pack(tbl)
str1 = mp.pack(tbl1)
st = str..str1
--offset, decoded = mp.unpack(st)
--t0 = utime()
--print(offset, decoded)
--util.ptable(decoded)
--offset1, decoded = mp.unpack(st, offset)
--print(offset1, decoded)
--util.ptable(decoded)
--print(utime()-t0)
--
--print(str)
--print(str1)
--print(#str, #str1, #st)
--t = mp.unpacker( st )
--local idx, val = t()
----print( tt, type(tt), at, type(at) )
--print('\nUnpacking '..idx)
--for i,v in pairs(val) do
--  print(i,v)
--end
--idx, val = t()
--print('\nUnpacking '..idx)
--for i,v in pairs(val) do
--  print(i,v)
--end
--idx, val = t()
--print('\nUnpacking ',idx)
--for i,v in pairs(val) do
--  print(i,v)
--end

--for k, v in mp.unpacker(t) do
--  print(k, v)
--end

--print(str:byte(1, #str))
--print(str1:byte(1, #str1))
--st = str..str1
--print(st)
--print(st:byte(1, #st))
----b = mp.unpack(str..str1)
--util.ptable(t)
--util.ptable(b)
--

