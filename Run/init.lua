cwd = cwd or os.getenv('PWD')
-- this module is used to facilitate interactive debuging

package.cpath = cwd.."/Lib/?.so;"..package.cpath;

package.path = cwd.."/Util/?.lua;"..package.path;
package.path = cwd.."/Config/?.lua;"..package.path;
package.path = cwd.."/Lib/?.lua;"..package.path;
package.path = cwd.."/Dev/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path;
package.path = cwd.."/Motion/keyframes/?.lua;"..package.path;
package.path = cwd.."/Motion/Walk/?.lua;"..package.path;
package.path = cwd.."/Motion/Arms/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/Test/?.lua;"..package.path;
package.path = cwd.."/?.lua;"..package.path;


local serialization = require('serialization')
local string = require('string')
local vector = require('vector')
local getch = require('getch')
local util = require('util')
local unix = require('unix')
local cutil = require('cutil')
local shm = require('shm')
