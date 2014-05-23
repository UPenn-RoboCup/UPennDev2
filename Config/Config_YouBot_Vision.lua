assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

------------
-- Vision --
------------
Config.vision = {}
Config.vision.wire = {
  ch = 'wire',
  rbox_ch = 'rbox' -- Robot guess bbox
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
  --focal_length = 184.75,
  focal_length = 320,
  hfov = 50 * DEG_TO_RAD,
  vfov = 50 * DEG_TO_RAD,
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
