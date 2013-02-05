local pwd = os.getenv('PWD')
package.cpath = pwd..'/../lib/qt/?.so;'..package.cpath
package.path = pwd..'/../lib/ffi/?.lua;'..package.path

local ffi = require 'ffi'
local libpng = require'libpng'

function yuyv2rgb(yuyv, w, h)
  local rgbimg = ffi.new('uint8_t[?]', w * h * 3)
  local countyuv = 0;
  local count = 0;
  for j = 0, h - 1 do
    for i = 0, w - 1 do
      local u = bit.rshift(bit.band(yuyv[countyuv], 0xFF000000), 24);
      local v = bit.rshift(bit.band(yuyv[countyuv], 0x0000FF00), 8);
      local y = bit.rshift(bit.band(yuyv[countyuv], 0x000000FF), 0);
      rgbimg[count] =     2 * (v - 128) + y -- R
      rgbimg[count + 1] = 2 * (u - 128) + y-- B
      rgbimg[count + 2] = y -- G
      rgbimg[count + 3] = rgbimg[count]
      rgbimg[count + 4] = rgbimg[count + 1]
      rgbimg[count + 5] = rgbimg[count + 2]

      count = count + 6
      countyuv = countyuv + 1;
    end
  end
  return rgbimg
end

Camera = require "OPCam"

uimage = Camera.get_image();
while (uimage == -1) do
  uimage = Camera.get_image()  
end
image = ffi.cast('uint32_t*', uimage) 
w = Camera.get_width()
h = Camera.get_height()
rgb = yuyv2rgb(image, w, h)

libpng.save('image.png', w, h, rgb)

Camera.stream_off();
Camera.stop();
print(Camera.get_width())
print(Camera.get_height())
