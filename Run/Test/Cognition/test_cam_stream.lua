Camera = require "uvcCam"
require 'Comm'
Comm.init('192.168.123.255', 54322);
require 'cjpeg'
require 'unix'

--while true do
for i=1,15 do
  local img = Camera.get_image();
  local jimg = cjpeg.compress(img);
  print('Sending compressed jpeg frame',i)
  Comm.send( jimg )
  unix.usleep(1e6)
end
Camera.stream_off();
Camera.stop();
