-- Add the required paths
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;

-- Require the right modules
local ffi = require 'ffi'
require 'ffi/torchffi'
local simple_ipc = require 'simple_ipc'
local libSlam = require 'libSlam'
local Sensors = require 'Config_Sensors'
require 'unix'

-- Setup IPC
local lidar_channel = simple_ipc.setup_subscriber('lidar');
local imu_channel = simple_ipc.setup_subscriber('imu');
local omap_channel = simple_ipc.setup_publisher('omap');

-- Poll multiple sockets
local channel_poll = simple_ipc.wait_on_channels( {lidar_channel, imu_channel} );

-- Initialize the map
local omap = libSlam.OMAP.data
local map_cdata = torch.data( omap )
local map_cdata_sz = omap:storage():size() * ffi.sizeof('char')
print('Map size',map_cdata_sz)

-- Reference the sensors
local ranges = Sensors.LIDAR0.ranges;
local ranges_cdata = torch.data( ranges )
local ranges_cdata_sz = ranges:storage():size() * ffi.sizeof('float')
print('Ranges size',ranges_cdata_sz)
local timestamp_cdata = ffi.new('double[1]',0);

-- Start the timing
local t = unix.time();
local t_last = t;
local map_rate = 15; --15Hz
local map_t = 1/(map_rate)
local channel_timeout = 2*map_t*1e3; -- milliseconds, or just wait (-1)
while true do
	
	local nevents = channel_poll:wait_on_any(channel_timeout)
	print("Number of events updated",nevents)
	
	if nevents>0 then
	-- Receive ipc sensor payload
	local nbytes, has_more = lidar_channel:receive(ranges_cdata, ranges_cdata_sz)
	-- Receive sensor timestamp
	if has_more then 
		local nbytes, has_more = lidar_channel:receive(timestamp_cdata)
		Sensors.LIDAR0.timestamp = timestamp_cdata[0]
	else
		print('No sensor timestamp received!')
	end
end
	--]]
	-- Process the first LIDAR
  --libSlam.processL0()
	-- Send the map a few times per second
	t = unix.time()
	if t-t_last>map_t then
		t_last = t;
		-- Send the payload
		omap_channel:send( map_cdata, map_cdata_sz, true );
		-- Send the timestamp
		omap_channel:send( libSlam.OMAP.timestamp )
		print('sending the map')
	end
end