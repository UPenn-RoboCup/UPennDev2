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
  width = 640,
  height = 480,
  fps = 30,
  focal_length = 369.5, --260,--184.75,
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
	Config.camera[1].width = 320
	Config.camera[1].height = 240
	Config.camera[1].focal_length = 320
end

return Config
