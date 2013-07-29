local handle = io.popen('pwd')
local cwd = handle:read("*a"):gsub("%s+$", "")
handle:close()
IS_WEBOTS = false
local HOME = cwd:gsub('Player.*$','')
HOME = HOME:gsub('Operate.*$','')
HOME = HOME:gsub('Tools.*$','')
HOME = HOME:gsub('Frameworks.*$','')
HOME = HOME:gsub('Util.*$','')
if HOME:find("Webots") ~= nil then
  HOME = HOME:gsub('Webots.*$','')
  IS_WEBOTS = true
end

KEYFRAME_DIR = HOME.."Player/Motion/keyframes"

OPERATING_SYSTEM = io.popen('uname'):read('*a'):lower():gsub("%s+$", "")
print( 'Working Dir:', cwd )
print( 'Home Dir:', HOME )
print( 'Operating Sys:', OPERATING_SYSTEM )
print( 'Keyframes directory:', KEYFRAME_DIR )

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

-- Include webots files, if needed
-- For now, these are located in Util
if IS_WEBOTS then
	print'Instantiating Webots specific items...'
end

-- include platform specific modules
local Config = require'Config'
PLATFORM_NAME = Config.platform.name
package.path = HOME..'Robots/'..PLATFORM_NAME..'/?.lua;'..package.path
package.cpath = HOME..'Robots/'..PLATFORM_NAME..'/?.so;'..package.cpath
