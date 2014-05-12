-- Include script to be run at the top of each file
-- This mainly sets the paths
-- It also adds very useful globals

-- Are we locally testing?
USE_LOCALHOST = true

-- Locate the Modules
CWD = assert(os.getenv('PWD'),'No PWD variable set!')
IS_WEBOTS = false
HOME = CWD:gsub('Player.*$','')
HOME = HOME:gsub('Robots.*$','')
HOME = HOME:gsub('Operate.*$','')
HOME = HOME:gsub('Modules.*$','')
HOME = HOME:gsub('Tools.*$','')
HOME = HOME:gsub('Util.*$','')
HOME = HOME:gsub('Test.*$','')
if HOME:find'Webots' ~= nil then
  HOME = HOME:gsub('Webots.*$','')
  IS_WEBOTS = true
	--print('Current path',package.path)
	package.path = './?.lua;/usr/local/share/lua/5.1/?.lua;/usr/local/share/lua/5.1/?/init.lua;/usr/local/lib/lua/5.1/?.lua;/usr/local/lib/lua/5.1/?/init.lua'
end

-- Useful constants
DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 180/math.pi

-- SJ: This removes the output buffer
io.stdout:setvbuf("no")

-- include C modules to cpath
-- For copying to the Nao and just dumping into one directory
package.cpath = HOME..'/Lib/?.so;'..package.cpath
-- getch.so is in Modules/getch/ (Modules/unix/unix.so -> Modules/?/?.so)
package.cpath = HOME..'/Modules/?/?.so;'..package.cpath
-- Sometimes there are helper lua files (even for ffi, but still experimental)
package.path = HOME..'/Modules/?.lua;'..package.path

-- include Lua utilities to path
package.path = HOME..'/Util/?.lua;'..package.path

-- include Shared Memory files to path
package.path = HOME..'/Memory/?.lua;'..package.path

-- include Robot Agnostic wrappers
package.path = HOME..'/Player/Dev/?.lua;'..package.path

-- include World files to the path
package.path = HOME..'/Player/World/?.lua;'..package.path

-- include Config files to path
package.path = HOME..'/Config/?.lua;'..package.path

-- Config is global now!
Config = require'Config'

-- include platform specific modules
ROBOT_HOME = HOME..'/Robots/'..Config.PLATFORM_NAME..'/'
package.path  = ROBOT_HOME..'?.lua;'..package.path
package.cpath = ROBOT_HOME..'?.so;'..package.cpath

KEYFRAME_DIR = HOME.."/Player/Keyframes"
LOG_DIR = HOME.."/Logs/"

-- Save the hostname
unix = require'unix'
HOSTNAME = unix.gethostname()
OPERATING_SYSTEM = unix.uname():lower()

-- Check for LuaJit and add to the global namespace
local ok, myffi = pcall(require,'ffi')
if ok then
	ffi = myffi
	C = ffi.C
end

-- Print out the globally available variables, when using include.lua
--[[
function print_env()
	print( 'Working Dir:', CWD )
	print( 'Home Dir:', HOME )
	print( 'Operating Sys:', OPERATING_SYSTEM )
	print( 'Host:', HOSTNAME )
	print( 'Webots:', IS_WEBOTS )
	print( 'Child thread:', IS_CHILD )
	print( 'Platform:', Config.PLATFORM_NAME )
	print( 'Keyframes directory:', KEYFRAME_DIR )
	print( 'package path:', package.path )
	print( 'package cpath:', package.cpath )
end
--]]
