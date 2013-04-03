local cwd = os.getenv('PWD')

local HOME = cwd:gsub('WebotsController.*$','')

-- include modules to cpath
package.cpath = HOME..'Frameworks/?/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/CArray/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/CUtil/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Jpeg/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Shm/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Unix/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/Util/Z/?.so;'..package.cpath

package.cpath = HOME..'Frameworks/Comm/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/ImageProc/?.so;'..package.cpath
package.cpath = HOME..'Frameworks/OccMap/?.so;'..package.cpath
