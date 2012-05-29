module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

computer = os.getenv('COMPUTER') or system;
if (string.find(computer, "Darwin")) then
   -- MacOS X uses .dylib:                                                      
   package.cpath = cwd.."/?.dylib;"..package.cpath;
   package.cpath = cwd.."/../../Player/Lib/?.dylib;"..package.cpath;
else
   package.cpath = cwd.."/?.so;"..package.cpath;
end

package.path = cwd.."/../../Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../Player/World/?.lua;"..package.path;

require("util")
require("ocm")
require("OccMap")
require("unix")

local time = unix.time();
print(time)
OccMap.init(50, 25, 40, time);
OccMap.vision_init(80);

width = 80;
free_bound = vector.zeros(width);
free_bound_type = vector.zeros(width);
for i = 1, width do
  free_bound[i] = i * 0.5434;
  free_bound_type[i] = i;
end
OccMap.vision_update(free_bound, free_bound_type, width, unix.time());
--[[
for i = 1, 50 do
  cur_time = unix.time();
  OccMap.time_decay(cur_time);
  OccMap.vision_update(OccMap.empty_userdata(width), 
                      OccMap.empty_userdata(width), width, cur_time);
end
--]]

-- Map Retrieve Test Case with matlab
occmap = OccMap.retrieve_map();
ocm.set_occ_map(occmap);

-- get map updated time
map_updated_time = OccMap.retrieve_map_updated_time();

--[[
-- Map related data retrieve test case
occdata = OccMap.retrieve_data();
util.ptable(occdata.robot_pos);
--]]

--occ.odometry_update(3,2,1);
--util.ptable(occmap.map)
--print(occmap.map[450]);
--[[
for i = 1 , 1 do
  x = math.random()/40;
  y = math.random()/40;
  a = math.random()/40;
  occ.odometry_update(x, y, a);
end
--]]

--cmap = ocm.get_occ_map();

