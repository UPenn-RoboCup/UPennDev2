-- Add the required paths
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;

local ffi = require 'ffi'
require 'ffi/torchffi'
local simple_ipc = require 'simple_ipc'

-- Setup IPC
local lidar_channel = simple_ipc.setup_subscriber('lidar');
local omap_channel = simple_ipc.setup_publisher('omap');

-- Initialize the map
--print('Map Size:',MAPS.sizex,MAPS.sizey)
--local map_cdata = torch.data( OMAP.data )

local ranges = torch.FloatTensor(1081);
local ranges_cdata = ffi.cast('float*',torch.data( ranges ) )
local timestamp_cdata = ffi.new('double[1]',0);

while true do
	-- Receive ipc sensor payload
	local nbytes, has_more = lidar_channel:receive(ranges_cdata, 1081*4)
	-- Receive sensor timestamp
	if has_more then 
		local nbytes, has_more = lidar_channel:receive(timestamp_cdata)
		timestamp = timestamp_cdata[0]
	else
		print('No sensor timestamp received!')
	end
  --processL0( SLAMMER, LIDAR0, IMU, OMAP, MAPS, xs )
  --omap_channel:send( map_cdata, OMAP.data_sz );
end