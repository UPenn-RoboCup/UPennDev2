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

require 'torch'
torch.Tensor = torch.DoubleTensor

require 'mapShift'
require 'processL0'

-- Default values
local MAPS = {}
MAPS.res        = .2;
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
--LIDAR0.angles  = ((0:LIDAR0.resd:(LIDAR0.nRays-1)*LIDAR0.resd)-135)*pi/180;
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

--LIDAR0.ranges = torch.Tensor(1081):fill(2);
--LIDAR0.ranges = torch.rand(LIDAR0.nRays):mul(3);
LIDAR0.ranges = torch.rand(LIDAR0.nRays):mul(.5)+3;
--LIDAR0.ranges = torch.range(1,20,19/LIDAR0.nRays);

IMU = {}
IMU.roll = 0;
IMU.pitch = -10 * math.pi/180; -- Look down a little
IMU.roll = 0;

--ranges = rcm.get_lidar_ranges();
--print( carray.get(ranges,1) )
for i=1,20 do
processL0( LIDAR0, IMU, OMAP, MAPS )
require 'map_io'
init_send()
-- Play it again Sam!
print('Sending map!')
send_map(OMAP.data)
end