local handle = io.popen('pwd')
local cwd = handle:read("*a"):gsub("%s+$", "")
handle:close()
local Webots = false
local HOME = cwd:gsub('Run.*$','')
HOME = HOME:gsub('Tools.*$','')
HOME = HOME:gsub('Frameworks.*$','')
HOME = HOME:gsub('Util.*$','')
if HOME:find("Webots") ~= nil then
  HOME = HOME:gsub('Webots.*$','')
  Webots = true
end
OPERATING_SYSTEM = io.popen('uname'):read('*a'):lower():gsub("%s+$", "")
print( 'Working Dir:', cwd )
print( 'Home Dir:', HOME )
print( 'Operating Sys:', OPERATING_SYSTEM )

-- include C modules to cpath
package.cpath = HOME..'Modules/?/?.so;'..package.cpath

-- include Lua utilities to path
package.path = HOME..'Util/?.lua;'..package.path

-- include Config files to path
package.path = HOME..'Config/?.lua;'..package.path

--[[
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
package.path = HOME..'Run/Motion/keyframes/?.lua;'..package.path
package.path = HOME..'Run/Motion/Walk/?.lua;'..package.path
package.path = HOME..'Run/Motion/Arms/?.lua;'..package.path
package.path = HOME..'Run/Config/?.lua;'..package.path
--]]

-- include webots stuff
if Webots then
  package.cpath = HOME..'Frameworks/Webots/Controller/?.so;'..package.cpath
  package.cpath = HOME..'Frameworks/Webots/GameControl/?.so;'..package.cpath
  package.path = HOME..'Frameworks/Webots/Comm/?.lua;'..package.path
  package.path = HOME..'Frameworks/Webots/GameControl/?.lua;'..package.path
end

-- include platform specific modules
local Config = require 'Config'
local Platform = dofile(HOME..'Platforms/'..Config.platform.name..'/Platform.lua')
for i = 1, #Platform.path do
  package.path = HOME..'Platforms/'..Config.platform.name..Platform.path[i]..package.path
end
for i = 1, #Platform.cpath do
  package.cpath = HOME..'Platforms/'..Config.platform.name..Platform.cpath[i]..package.cpath
end
