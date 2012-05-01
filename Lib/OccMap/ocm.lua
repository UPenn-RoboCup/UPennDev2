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

require("Config")
require("util")
require("shm")

mapsize = 50;

shared = {}
shsize = {}

shared.occ = {};
shared.occ.map = vector.zeros(mapsize * mapsize);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);


