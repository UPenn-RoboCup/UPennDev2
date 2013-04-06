dofile('Run/include.lua')
local Body = require 'Body'
local Camera = require 'Camera'
local cjpeg = require 'cjpeg'
local carray = require 'carray'
local msgpack = require 'msgpack'
local simple_ipc = require 'simple_ipc'

local webots_version = io.popen('webots --version', 'r')
wb_version_str = webots_version:read('*a')
wb_version_str = wb_version_str:match('%d.%d.%d')
wb_version = (wb_version_str:byte(1)-48) * 100 + (wb_version_str:byte(3)-48) * 10
                + wb_version_str:byte(5)-48
print('Webots Version '..wb_version)
webots_version:close()

print('Loading the Webots controller script!')
local camera_channel = simple_ipc.new_publisher('camera')
local t_last = Body.get_time()
print(t_last)
local debug_fps = 4
local inv_debug_fps = 1/debug_fps
while true do
  local t = Body.get_time()
  local t_diff = t - t_last;

  -- Enable debugging
  if (t-t_last)>inv_debug_fps then
--    print('Debugging...')
    t_last = t
  end
--
--  -- Grab the image and send it away for processing...
  local image = carray.byte( Camera.get_image(), Camera.get_width() * Camera.get_height() * 3 )
  local jimage = cjpeg.compress( Camera.get_image(), Camera.get_width(), Camera.get_height(), 3)
  local img_str = tostring(image)
--  --local img_str = msgpack.pack(image, Camera.get_width() * Camera.get_height() * 3)
--  print(#jimage, type(Camera.get_image()), type(image), type(img_str) )
--  print(Camera.get_width(), Camera.get_height() )

  -- Send data on IPC
--  local res = camera_channel:send('hello')
  local res = camera_channel:send(img_str)
  -- Update the Webots timestamp and motor commands
  Body.update()
  io.stdout:flush();
end
