local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../../Player/Util/ffi/?.lua;'..package.path

require 'unix'
require 'cpng'

t0 = unix.time()
local p = cpng.new('small.png')
print(unix.time() - t0)

t0 = unix.time()
p:write('iii.png')
print(unix.time() - t0)
print(p)
