package.cpath = '../unix/?.so;'..package.cpath
package.path = '../../Util/?.lua;'..package.path
require'unix'
local torch = require 'torch'
local util = require 'util'
local msgpack = require 'msgpack'

torch.Tensor = torch.DoubleTensor
local aa = torch.rand(10000)
print(aa[1])
t0 = unix.time()
raw_str = msgpack.pack(aa)
print('raw pack 10000 torch tensor:', unix.time() - t0, #raw_str)

t0 = unix.time()
table_str = msgpack.pack(torch.Tensor({100, 100, 100}))
tbl = msgpack.unpack(table_str)
print('table unpack:', unix.time() - t0)
--util.ptable(tbl)

t0 = unix.time()
t_rand = msgpack.unpack(raw_str, 'torch', 'double')
t_tbl  = msgpack.unpack(table_str, 'torch', 'double')
print('torch unpack:', unix.time() - t0)

print(t_rand[1])
