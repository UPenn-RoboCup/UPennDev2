package.cpath = '../../Player/Lib/?.so;'..package.cpath
require 'msgpack'
require 'carray'

msgpack.pack(432.543)
msgpack.pack(true)
msgpack.pack('hellp world')

local t = {}
t.ast = 'fsfa'
t.brg = 32423
t[1] = 3445
msgpack.pack(t)

local udata = carray.byte('this is a test')
msgpack.pack(udata)
msgpack.pack(udata:pointer())

print('all together\n')

msgpack.pack(432.543,true,'hellp world',t,udata, udata:pointer())


