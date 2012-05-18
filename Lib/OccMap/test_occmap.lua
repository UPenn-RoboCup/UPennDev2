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
occ = require("OccMap")

occmap = occ.retrieve();

util.ptable(occmap.robot_pos)
--print(occmap.map[450]);


--ocm.set_occ_map(occmap.map);

--cmap = ocm.get_occ_map();

