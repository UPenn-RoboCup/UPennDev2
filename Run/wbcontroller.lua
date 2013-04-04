local cwd = os.getenv('PWD')
dofile('Run/include.lua')

local unix = require 'unix'
local Body = require 'Body'
local Camera = require 'Camera'
local cjpeg = require 'cjpeg'
local carray = require 'carray'
local msgpack = require 'msgpack'

--local simple_ipc = require 'simple_ipc'

--local camera_channel = simple_ipc.setup_publisher('camera')

local last_update = Body.get_time()
local update_interval = 0.01
local last_vision_update = Body.get_time()
local vision_update_interval = 0.04; --25fps update

print('load')

while true do
  if (unix.time() - last_update) > update_interval then
--    print(unix.time())
    last_update = Body.get_time()
  end

  if (unix.time() - last_vision_update) > vision_update_interval then
    local image = carray.byte(Camera.get_image(), Camera.get_width() * Camera.get_height() * 3)
    local img_str = tostring(image)
    print(Camera.get_width(), Camera.get_height(), #img_str)
--    camera_channel:send('hello')
--    unix.usleep(1e6)
--    local img_str = msgpack.pack(image, Camera.get_width() * Camera.get_height() * 3)
    last_vision_update = Body.get_time()
  end
end

