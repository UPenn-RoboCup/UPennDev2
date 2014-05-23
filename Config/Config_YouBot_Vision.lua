assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

------------
-- Vision --
------------
Config.vision = {}
Config.vision.wire = {
  ch = 'wire',
}

-------------
-- Cameras --
-------------
Config.camera = {}

table.insert(Config.camera,
{
  name = 'larm',
  dev = '/dev/video1',
  unreliable = 33333,
  reliable = 33333,
  ch = 'camera'..(#Config.camera + 1),
  fmt = 'yuyv',
  width = 320,
  height = 240,
  fps = 30,
  focal_length = 184.75,
  param = {
    {'Brightness', 100},
    {'Contrast', 25},
    {'Saturation', 190},
  },
  detection_pipeline = {
    'detectWire'
  }
})

-- Webots override
if IS_WEBOTS then

end

return Config
