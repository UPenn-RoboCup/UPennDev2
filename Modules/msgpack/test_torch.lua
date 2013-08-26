package.cpath = '../unix/?.so;'..package.cpath
package.path = '../../Util/?.lua;'..package.path
local torch = require 'torch'
local util = require 'util'
local msgpack = require 'msgpack'

torch.Tensor = torch.DoubleTensor
local aa = torch.rand(100, 100, 100)
t0 = unix.time()
array_str = msgpack.pack(aa)
print('table pack 100x100x100 torch tensor:', unix.time() - t0, #array_str)
t0 = unix.time()
raw_str = msgpack.pack(aa, 'raw')
print('raw pack 100x100x100 torch tensor:', unix.time() - t0, #raw_str)

t0 = unix.time()
table_str = msgpack.pack(torch.Tensor({100, 100, 100}))
tbl = msgpack.unpack(table_str)
print('table unpack:', unix.time() - t0)
--util.ptable(tbl)

t0 = unix.time()
th = msgpack.unpack(table_str, 'torch', 'double')
print('torch unpack:', unix.time() - t0)
