-- Add the required paths
cwd = '.';
uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/../?.so;"..package.cpath;
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd.."/../Config/?.lua;"..package.path;
package.path = cwd.."/../Vision/?.lua;"..package.path;

require('serialization');
require('util');
require('unix');
require('cutil');
require('vector');
local ffi = require 'ffi'
require 'torch'
require 'ffi/torchffi'
torch.Tensor = torch.DoubleTensor

require 'mapShift'
require 'processL0'

-- Default values
local MAPS = {}
MAPS.res        = .1;
MAPS.invRes     = 1/MAPS.res;
MAPS.windowSize = 10; -- meters to see 
MAPS.edgeProx   = 2;
MAPS.xmin       = 0 - MAPS.windowSize;
MAPS.ymin       = 0 - MAPS.windowSize;
MAPS.xmax       = 0 + MAPS.windowSize;
MAPS.ymax       = 0 + MAPS.windowSize;
MAPS.zmin       = 0;
MAPS.zmax       = 5;
MAPS.sizex  = (MAPS.xmax - MAPS.xmin) / MAPS.res + 1;
MAPS.sizey  = (MAPS.ymax - MAPS.ymin) / MAPS.res + 1;

-- Occupancy Map
local OMAP = {}
OMAP.res        = MAPS.res;
OMAP.invRes     = MAPS.invRes;
OMAP.xmin       = MAPS.xmin;
OMAP.ymin       = MAPS.ymin;
OMAP.xmax       = MAPS.xmax;
OMAP.ymax       = MAPS.ymax;
OMAP.zmin       = MAPS.zmin;
OMAP.zmax       = MAPS.zmax;
OMAP.sizex  = MAPS.sizex;
OMAP.sizey  = MAPS.sizey;
--OMAP.data = torch.Tensor(OMAP.sizex,OMAP.sizex):zero()
OMAP.data = torch.ByteTensor(OMAP.sizex,OMAP.sizex)

-- Helper Functions

--print(OMAP.data)
--mapShift( OMAP, 5*OMAP.res, 5*OMAP.res )
--print(OMAP.data)

--
-- Test LIDAR processing
--
--
LIDAR0 = {}
LIDAR0.resd    = 0.25;
LIDAR0.res     = LIDAR0.resd/180*math.pi;
LIDAR0.nRays   = 1081;
LIDAR0.angles  = torch.range(0,(LIDAR0.nRays-1)*LIDAR0.resd,LIDAR0.resd)
LIDAR0.angles = (LIDAR0.angles - 135) * math.pi/180
LIDAR0.cosines = torch.cos(LIDAR0.angles);
LIDAR0.sines   = torch.sin(LIDAR0.angles);
LIDAR0.offsetx = 0.137;
LIDAR0.offsety = 0;
LIDAR0.offsetz = 0.54;  --from the body origin (not floor)

LIDAR0.mask    = torch.Tensor(LIDAR0.angles:size()):fill(1);
-- Be very conservative to ensure no interpolated antenna obstacles
--LIDAR0.mask(1:190) = 0;
--LIDAR0.mask(end-189:end) = 0;
LIDAR0.present = 1;
LIDAR0.startTime = 0;
LIDAR0.ranges = torch.FloatTensor():rand(LIDAR0.nRays):mul(.5)+3;
if not LIDAR0.ranges:isContiguous() then
	print('Ranges are not contiguous!')
	return;
end


IMU = {}
IMU.roll = 0;
IMU.pitch = 0;-- -10 * math.pi/180; -- Look down a little
IMU.roll = 0;

require 'rcm'
--local ranges = ffi.new('float[?]',LIDAR0.nRays)
local ranges_cdata = torch.data( LIDAR0.ranges )

--print( carray.get(ranges,1) )
print('Map Size:',MAPS.sizex,MAPS.sizey)
--for i=1,20 do
while true do
	ffi.copy(ranges_cdata,rcm.get_lidar_ranges(),LIDAR0.nRays);
--	print(ranges_cdata[1],LIDAR0.ranges[1])
  --LIDAR0.ranges = torch.rand(LIDAR0.nRays):mul(.5)+3;
  processL0( LIDAR0, IMU, OMAP, MAPS )
	unix.usleep(1e5)
--	if(i==10) then mapShift( OMAP, 10*OMAP.res, 10*OMAP.res ) end
end
