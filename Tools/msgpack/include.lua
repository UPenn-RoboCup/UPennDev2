local pwd = os.getenv('PWD')
local repopath = '../../Player'
package.path = repopath..'/Util/?.lua;'..package.path
package.cpath = repopath..'/Lib/?.so;'..package.cpath

package.cpath = pwd..'/lib/GeographicLib/?.so;'..package.cpath
package.cpath = pwd..'/lib/Serial/?.so;'..package.cpath
package.cpath = pwd..'/lib/LuaXml/?.so;'..package.cpath
package.path = pwd..'/lib/LuaXml/?.lua;'..package.path


