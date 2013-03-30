package.cpath = '../../Player/Lib/?.so;'..package.cpath
package.path = '../../Player/Util/?.lua;'..package.path

require 'msgpack'
require 'carray'
local mp = require 'MessagePack'
local util = require 'util'

str = msgpack.pack(a)
strmp = mp.pack(a)
print(msgpack.unpack(strmp))
print(#str)

str = msgpack.pack(432.543)
strmp = mp.pack(432.543)
print(msgpack.unpack(strmp))

strmp = mp.pack(-432)
print(msgpack.unpack(strmp))

print(#str)
str = msgpack.pack(true)
strmp = mp.pack(true)
msgpack.unpack(strmp)
print(#str)
str = msgpack.pack('hellp world')
strmp = mp.pack('hellp world')
print(msgpack.unpack(strmp))
print(#str)

print('test array')
str = msgpack.pack({4,5,6,7})
strmp = mp.pack({4,5,6,7})
util.ptable(msgpack.unpack(strmp))
--msgpack.unpack(strmp)

local t = {}
t.ast = 'fsfa'
t.brg = 32423
t[1] = 3445
t[3] = 34
str = msgpack.pack(t)
strmp = mp.pack(t)
util.ptable(msgpack.unpack(strmp))
print(#str)

str = msgpack.pack({4,5,6,t})
strmp = mp.pack({4,5,6,t})
util.ptable(msgpack.unpack(strmp))

--local udata = carray.byte('this is a test')
--str = msgpack.pack(udata)
--strmp = mp.pack(udata)
--msgpack.unpack(strmp)
--print(#str)
--str = msgpack.pack(udata:pointer())
--strmp = mp.pack(udata:pointer())
--msgpack.unpack(strmp)
--print(#str)

print('all together\n')

--str = msgpack.pack(432.543,true,'hellp world',t,udata, udata:pointer())
--print(#str)


