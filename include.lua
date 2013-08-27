local handle = io.popen('pwd')
CWD = handle:read("*a"):gsub("%s+$", "")
handle:close()
IS_WEBOTS = false
HOME = CWD:gsub('Player.*$','')
HOME = HOME:gsub('Operate.*$','')
HOME = HOME:gsub('Tools.*$','')
HOME = HOME:gsub('Frameworks.*$','')
HOME = HOME:gsub('Util.*$','')
if HOME:find("Webots") ~= nil then
  HOME = HOME:gsub('Webots.*$','')
  IS_WEBOTS = true
end

KEYFRAME_DIR = HOME.."Player/Keyframes"
OPERATING_SYSTEM = io.popen('uname'):read('*a'):lower():gsub("%s+$", "")

-- include C modules to cpath
-- getch.so is in Modules/getch/ (Modules/unix/unix.so -> Modules/?/?.so)
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
--[[
package.path = HOME..'Player/Motion/?.lua;'..package.path
package.path = HOME..'Player/Motion/keyframes/?.lua;'..package.path
package.path = HOME..'Player/Motion/Walk/?.lua;'..package.path
package.path = HOME..'Player/Motion/Arms/?.lua;'..package.path
--]]

-- include World files to the path
package.path = HOME..'Player/World/?.lua;'..package.path

-- Include webots files, if needed
-- For now, these are located in Util
if IS_WEBOTS then print'Instantiating Webots specific items...' end

-- include platform specific modules
local Config = require'Config'
PLATFORM_NAME = Config.PLATFORM_NAME
package.path  = HOME..'Robots/'..PLATFORM_NAME..'/?.lua;'..package.path
package.cpath = HOME..'Robots/'..PLATFORM_NAME..'/?.so;'..package.cpath

-- Print out the globally available variables, when using include.lua
print( 'Working Dir:', CWD )
print( 'Home Dir:', HOME )
print( 'Operating Sys:', OPERATING_SYSTEM )
print( 'Keyframes directory:', KEYFRAME_DIR )
print( 'Webots:', IS_WEBOTS )
print( 'Platform:', PLATFORM_NAME )