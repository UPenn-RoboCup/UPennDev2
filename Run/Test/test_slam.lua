dofile('../include.lua')

-- Require the right modules
local Sensors = require 'Config_Sensors'
local simple_ipc = require 'simple_ipc'
local ffi = require 'ffi'
require 'ffi/torchffi'
local libSlam = require 'libSlam'
local mp = require 'ffi/msgpack'
require 'unix'

-- Reference the sensors
local ranges = Sensors.LIDAR0.ranges;
local ranges_cdata = torch.data( ranges )
local ranges_cdata_sz = ranges:storage():size() * ffi.sizeof('float')
local timestamp_cdata = ffi.new('double[1]',0);
local imu_buf_sz = 100;
local imu_buf = ffi.new('char[?]',imu_buf_sz)

-- Initialize the map
local omap = libSlam.OMAP.data
local map_cdata = torch.data( omap )
local map_cdata_sz = omap:storage():size() * ffi.sizeof('char')
print('Map size:',libSlam.MAPS.sizex,libSlam.MAPS.sizey)

-- Setup IPC
local lidar_channel = simple_ipc.setup_subscriber('lidar');
local lidar_callback = function()
  -- Receive ipc sensor payload
  local nbytes, has_more = lidar_channel:receive(ranges_cdata, ranges_cdata_sz)
  -- Receive sensor timestamp
  if has_more then 
    local nbytes, has_more = lidar_channel:receive(timestamp_cdata)
    Sensors.LIDAR0.timestamp = timestamp_cdata[0]
  else
    print('No lidar timestamp received!')
  end
end
lidar_channel.callback = lidar_callback

local imu_channel = simple_ipc.setup_subscriber('imu');
imu_channel.callback = function()
	local nbytes, has_more = imu_channel:receive(imu_buf, imu_buf_sz)
	local offset, decoded = mp.unpack( ffi.string(imu_buf,nbytes) )
	--print(decoded.t)
end

local omap_channel = simple_ipc.setup_publisher('omap');
-- Poll multiple sockets
local wait_channels = {lidar_channel, imu_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

-- Start the timing
local t = unix.time();
local t_last = t;
local map_rate = 15; --15Hz
local map_t = 1/(map_rate)
local channel_timeout = 2*map_t*1e3; -- milliseconds, or just wait (-1)
local t_last_lidar = Sensors.LIDAR0.timestamp;
while true do

  local nevents, event_ids = channel_poll:wait_on_any(channel_timeout)
  --print("Number of events updated",nevents, unpack(event_ids))

  if nevents>0 then
		for i=1,#event_ids do
			wait_channels[ event_ids[i] ]:callback()
		end
		-- Process upon receiving data
		if Sensors.LIDAR0.timestamp > t_last_lidar then
			t_last_lidar = Sensors.LIDAR0.timestamp
			libSlam.processL0()
		end
  end
	
  -- Send the map at set intervals
  t = unix.time()
  if t-t_last>map_t then
    t_last = t;
    -- Send the payload
    omap_channel:send( map_cdata, map_cdata_sz, true );
    -- Send the timestamp
    omap_channel:send( libSlam.OMAP.timestamp )
    --print('sending the map')
  end
end
