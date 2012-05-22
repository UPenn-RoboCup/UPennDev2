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

require("util")
--require("ocm")
require("OccMap")

occmap = OccMap.map();

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
--ocm.set_occ_map(occmap.map);

--cmap = ocm.get_occ_map();

