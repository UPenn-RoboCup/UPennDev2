local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../Player/Util/ffi/?.lua;'..package.path
package.path = pwd..'/../../Player/Util/?.lua;'..package.path

local simple_ipc = require 'simple_ipc'
local imu_channel = simple_ipc.new_publisher('imu');
local mp = require 'MessagePack'
local unix = require 'unix'

while (1) do
  t = {}
  t.a = 354522
  t.time = unix.time()
  str = 'fafafa'
  imu_channel:send({mp.pack(t), mp.pack(t)});
  print('send table '..unix.time())
  unix.usleep(1e6 * 0.01)
end
