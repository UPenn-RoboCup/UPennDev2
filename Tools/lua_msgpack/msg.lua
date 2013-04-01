package.cpath = '../../Player/Lib/?.so;'..package.cpath
package.path = '../../Player/Util/?.lua;'..package.path

require 'msgpack'
require 'carray'
local mp = require 'cmsgpack'
local util = require 'util'
require 'unix'

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
t0 = unix.time()
str = msgpack.pack({4,5,6,7})
print(unix.time() - t0);
t0 = unix.time()
strmp = mp.pack({4,5,6,7})
print(unix.time() - t0);
--util.ptable(msgpack.unpack(strmp))
print('size '..#str, #strmp)
tbl = msgpack.unpack(str)

util.ptable(tbl)
--
print('test array')
local t = {}
t.ast = 'fsfa'
t.brg = 32423
t[1] = 3445
t[3] = 34
t0 = unix.time()
str = msgpack.pack(t)
print(unix.time() - t0);
t0 = unix.time()
strmp = mp.pack(t)
print(unix.time() - t0);
print('size '..#str, #strmp)
tbl = msgpack.unpack(str)
util.ptable(tbl)

--strmp = mp.pack(t)
--util.ptable(msgpack.unpack(strmp))
--print(#str)
--
--str = msgpack.pack({4,5,6,t})
--strmp = mp.pack({4,5,6,t})
--util.ptable(msgpack.unpack(strmp))
--
----local udata = carray.byte('this is a test')
----str = msgpack.pack(udata)
----strmp = mp.pack(udata)
----msgpack.unpack(strmp)
----print(#str)
----str = msgpack.pack(udata:pointer())
----strmp = mp.pack(udata:pointer())
----msgpack.unpack(strmp)
----print(#str)
--
--print('all together\n')
--
----str = msgpack.pack(432.543,true,'hellp world',t,udata, udata:pointer())
----print(#str)
--
--filename = '../../../bus/data/stateMP-03.29.2013.13.58.27-0'
--file = io.open(filename, 'r+')
--str = file:read('*a')
--print(#str)
--up = msgpack.unpacker(str)
--ret = up:unpack()
--local c = 0
--while ret do
----  util.ptable(ret)
--  c = c + 1
--  ret = up:unpack()
--end
--print(c)
