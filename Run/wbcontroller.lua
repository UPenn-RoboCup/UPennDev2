local cwd = os.getenv('PWD')
dofile('Run/include.lua')

--local unix = require 'unix'
local Body = require 'Body'
local Camera = require 'Camera'
local cjpeg = require 'cjpeg'
local carray = require 'carray'
local msgpack = require 'msgpack'

local simple_ipc = require 'simple_ipc'

local camera_channel = simple_ipc.setup_publisher('camera')

print('load')
local t_last = Body.get_time()
local debug_fps = 4
local inv_debug_fps = 1/debug_fps
while true do
  local t = Body.get_time()
  local t_diff = t - t_last;
  if (t-t_last)>inv_debug_fps then
    print(t,t_last)
    t_last = t
  end

    local image = carray.byte( Camera.get_image(), Camera.get_width() * Camera.get_height() * 3 )
    --local img_str = tostring(image)
    print( type(Camera.get_image()), type(image), type(img_str) )
    print(Camera.get_width(), Camera.get_height() )
    local res = camera_channel:send('hello')
    print(res)
--    local img_str = msgpack.pack(image, Camera.get_width() * Camera.get_height() * 3)
  Body.update()
end

