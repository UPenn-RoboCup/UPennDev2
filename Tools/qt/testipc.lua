local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../Player/Util/ffi/?.lua;'..package.path
package.path = pwd..'/../../Player/Util/?.lua;'..package.path

local simple_ipc = require 'simple_ipc'
local imu_channel = simple_ipc.setup_subscriber('imu');

while (1) do
  local imu_data, has_more = imu_channel:receive();
  print(imu_data)
end
