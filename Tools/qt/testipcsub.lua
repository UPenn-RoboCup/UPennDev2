local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../Player/Util/ffi/?.lua;'..package.path
package.path = pwd..'/../../Player/Util/?.lua;'..package.path

local simple_ipc = require 'simple_ipc'
local imu_channel = simple_ipc.setup_publisher('imu');
local unix = require 'unix'

while (1) do
--  imu_channel:send(tostring(unix.time()));
  imu_channel:send('fdfdfd');
  unix.sleep(1e6)
end
