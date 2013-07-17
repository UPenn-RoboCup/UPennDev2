local handle = io.popen('pwd')
local cwd = handle:read("*a"):gsub("%s+$", "")
handle:close()
local Webots = false
local HOME = cwd:gsub('Player.*$','')
HOME = cwd:gsub('Operate.*$','')
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

-- include Shared Memory files to path
package.path = HOME..'Memory/?.lua;'..package.path

-- include Robot Agnostic wrappers
package.path = HOME..'Player/Dev/?.lua;'..package.path

-- include Motion files to path
package.path = HOME..'Player/Motion/?.lua;'..package.path
package.path = HOME..'Player/Motion/keyframes/?.lua;'..package.path
package.path = HOME..'Player/Motion/Walk/?.lua;'..package.path
package.path = HOME..'Player/Motion/Arms/?.lua;'..package.path

-- include World files to the path
package.path = HOME..'Player/World/?.lua;'..package.path



--[[
package.path = HOME..'Player/?.lua;'..package.path
package.path = HOME..'Player/GameFSM/?.lua;'..package.path
package.path = HOME..'Player/Lib/?.lua;'..package.path
package.path = HOME..'Player/Test/?.lua;'..package.path
package.path = HOME..'Player/Interaction/?.lua;'..package.path
package.path = HOME..'Player/Vision/?.lua;'..package.path
package.path = HOME..'Player/HeadFSM/?.lua;'..package.path
package.path = HOME..'Player/Slam/?.lua;'..package.path
package.path = HOME..'Player/Log/?.lua;'..package.path
package.path = HOME..'Player/Dev/?.lua;'..package.path
package.path = HOME..'Player/BodyFSM/?.lua;'..package.path
package.path = HOME..'Player/Util/?.lua;'..package.path
package.path = HOME..'Player/Config/?.lua;'..package.path
--]]

-- include webots stuff
if Webots then
	print'Instantiating Webots specific items...'
  package.cpath = HOME..'Frameworks/Webots/Controller/?.so;'..package.cpath
  package.cpath = HOME..'Frameworks/Webots/GameControl/?.so;'..package.cpath
  package.path = HOME..'Frameworks/Webots/Comm/?.lua;'..package.path
  package.path = HOME..'Frameworks/Webots/GameControl/?.lua;'..package.path
end

-- include platform specific modules
local Config = require'Config'
local Platform = dofile(HOME..'Robots/'..Config.platform.name..'/Platform.lua')
for i = 1, #Platform.path do
  package.path = HOME..'Robots/'..Config.platform.name..Platform.path[i]..package.path
end
for i = 1, #Platform.cpath do
  package.cpath = HOME..'Robots/'..Config.platform.name..Platform.cpath[i]..package.cpath
end
