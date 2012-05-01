module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

computer = os.getenv('COMPUTER') or system;
if (string.find(computer, "Darwin")) then
   -- MacOS X uses .dylib:                                                      
   package.cpath = cwd.."/?.dylib;"..package.cpath;
else
   package.cpath = cwd.."/?.so;"..package.cpath;
end

package.path = cwd.."/../../Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../Player/Util/?.lua;"..package.path;

require("ocm")
occ = require("OccMap")

occ.init(50)

map = occ.retrieve();
ocm.set_occ_map(map);

cmap = ocm.get_occ_map();
for k, v in ipairs(cmap) do 
	print(k, v);
end


