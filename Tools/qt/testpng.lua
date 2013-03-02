local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/qt/?.so;'..package.cpath
package.path = pwd..'/lib/ffi/?.lua;'..package.path

local ffi = require 'ffi'
local libpng = require 'libpng'
require 'carray'

fname = 'Image-1-10.png'
imgload = libpng.load({path = fname})
--print(imgload.w, imgload.h)

--print(imgload.data, imgload.data[1000])
for k, v in pairs(imgload) do
  print(k, v)
end

bb = carray.byte(imgload.data, imgload.stride * imgload.h)
print(bb[1000], bb[1001])
print(bb:__len())
