local cwd = os.getenv('PWD')
dofile(cwd.."/Player/include.lua")

local unix = require 'unix'
local Body = require 'Body'
local Camera = require 'Camera'

local simple_ipc = require 'simple_ipc'

local last_update = unix.time()
local update_interval = 0.01
local last_vision_update = unix.time()
local vision_update_interval = 0.04; --25fps update

while true do
  if (unix.time() - last_update) > update_interval then
--    print(unix.time())
    last_update = unix.time()
  end

  if (unix.time() - last_vision_update) > vision_update_interval then
    local image = Camera.get_image()
    print('update vision', image, unix.time())
    last_vision_update = unix.time()
  end
end
