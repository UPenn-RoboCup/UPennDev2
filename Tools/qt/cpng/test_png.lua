local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../../Player/Util/ffi/?.lua;'..package.path

require 'unix'
require 'cpng'

t0 = unix.time()
local p = cpng.new('small.png')
print(unix.time() - t0)

print('width  '..p:width())
print('height  '..p:height())
print('stride  '..p:stride())
print('color type  '..p:color_type())
print('bit depth  '..p:bit_depth())
print('data lenght  '..p:__len())

for i = 1, p:__len(), 3 do
  print(p[i], p[i+1], p[i+2])
end
print(p[1920]) -- start from 1
print(p[1]) -- start from 1

t0 = unix.time()
p:write('iii.png')
--print(unix.time() - t0)
--print(p)
