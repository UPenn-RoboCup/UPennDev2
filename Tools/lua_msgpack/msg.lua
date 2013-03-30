package.cpath = '../../Player/Lib/?.so;'..package.cpath
package.path = '../../Player/Util/?.lua;'..package.path

require 'msgpack'
require 'carray'
local mp = require 'MessagePack'

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

local t = {}
t.ast = 'fsfa'
t.brg = 32423
t[1] = 3445
str = msgpack.pack(t)
strmp = mp.pack(t)
msgpack.unpack(strmp)
print(#str)

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


