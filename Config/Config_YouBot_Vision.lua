assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

-------------
-- Cameras --
-------------
Config.camera = {}

table.insert(Config.camera,
{
  name = 'larm',
  dev = '/dev/video1',
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

return Config
