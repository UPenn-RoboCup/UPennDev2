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

local unix = require 'unix'

-- include modules to path
package.path = HOME..'Run/?.lua;'..package.path

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
