local pwd = os.getenv('PWD')
package.cpath = pwd..'/../lib/qt/?.so;'..package.cpath
package.path = pwd..'/../lib/ffi/?.lua;'..package.path

local ffi = require 'ffi'
local libpng = require'libpng'
local Camera = require "OPCam"
--local unix = require 'unix'

rgbimg = ffi.new('uint8_t[?]', 640 * 480 * 3)
function yuyv2rgb(rgbimg, yuyv, w, h)
--  local rgbimg = {}
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
  print('converted')
  return rgbimg
end

ffi.cdef[[
  int poll(struct pullfd *fds, unsigned long nfds, int timeout);
]]

imagecounter = 0
for imagecounter = 0, 30 do
  uimage = Camera.get_image();
  while (uimage == -1) do
    ffi.C.poll(nil, 0, 33)
    uimage = Camera.get_image()  
    print(uimage, imagecounter)
  end
  w = Camera.get_width()
  h = Camera.get_height()
  img = ffi.cast('uint32_t*', uimage) 
--  img = ffi.cast('uint8_t*', uimage) 
--  ffi.copy(image, img, w * h * 3)
--  print('grab image')
  rgb = yuyv2rgb(rgbimg, img, 640, 480)
--  rgbdata = ffi.new('uint8_t[?]', w * h * 3, rgb)
--  ffi.copy(rgb, rgbimg, w * h * 3)
  imagecounter = imagecounter + 1
  libpng.save('image'..imagecounter..'.png', w, h, rgb)
--  libpng.save('image'..imagecounter..'.png', w, h, img)
  ffi.C.poll(nil,0, 33)

  print("image saved")
end

Camera.stream_off();
Camera.stop();
print(Camera.get_width())
print(Camera.get_height())
