local cwd = os.getenv('PWD')

local HOME = cwd:gsub('WebotsController.*$','')

-- include modules to cpath
package.cpath = HOME..'Frameworks/?/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/CArray/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/CUtil/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Shm/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Unix/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Z/?.so;'..package.cpath

package.cpath = HOME..'Frameworks/Comm/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/ImageProc/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/OccMap/?.so;'..package.cpath

-- include modules to path
package.path = HOME..'Run/?.lua;'..package.path
package.path = HOME..'Run/GameFSM/?.lua;'..package.path
package.path = HOME..'Run/Lib/?.lua;'..package.path
package.path = HOME..'Run/Test/?.lua;'..package.path
package.path = HOME..'Run/Interaction/?.lua;'..package.path
package.path = HOME..'Run/Vision/?.lua;'..package.path
package.path = HOME..'Run/HeadFSM/?.lua;'..package.path
package.path = HOME..'Run/World/?.lua;'..package.path
package.path = HOME..'Run/Slam/?.lua;'..package.path
package.path = HOME..'Run/Log/?.lua;'..package.path
package.path = HOME..'Run/Dev/?.lua;'..package.path
package.path = HOME..'Run/BodyFSM/?.lua;'..package.path
package.path = HOME..'Run/Util/?.lua;'..package.path
package.path = HOME..'Run/Motion/?.lua;'..package.path
package.path = HOME..'Run/Config/?.lua;'..package.path

-- include platform specific modules
local Config = require 'Config'
local Platform = dofile(HOME..'Platforms/'..Config.platform.name..'/Platform.lua')
for i = 1, #Platform.path do
  package.path = HOME..'Platforms/'..Config.platform.name..Platform.path[i]..package.path
end
for i = 1, #Platform.cpath do
  package.cpath = HOME..'Platforms/'..Config.platform.name..Platform.cpath[i]..package.cpath
end
