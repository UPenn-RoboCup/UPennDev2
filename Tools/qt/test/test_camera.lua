local pwd = os.getenv('PWD')
package.cpath = pwd..'/../lib/qt/?.so;'..package.cpath
package.path = pwd..'/../lib/ffi/?.lua;'..package.path

local ffi = require 'ffi'
local libpng = require'libpng'

Camera = require "OPCam"

uimage = Camera.get_image();
while (uimage == -1) do
  uimage = Camera.get_image()  
end

print(uimage)
image = ffi.cast('uint8_t*', uimage) 
w = Camera.get_width()
h = Camera.get_height()
libpng.save('image.png', w, h, image)

Camera.stream_off();
Camera.stop();
print(Camera.get_width())
print(Camera.get_height())
