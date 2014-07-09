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
  udp_port = 33333,
  ch = 'camera'..(#Config.camera + 1),
  fmt = 'yuyv',
  width = 320,
  height = 240,
  fps = 30,
  focal_length = 500, --400, --380, --320, --240, --200, --183.75,
  hfov = 50 * DEG_TO_RAD,
  vfov = 50 * DEG_TO_RAD,
  auto_param = {
  },
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
	Config.camera[1].w = 320
	Config.camera[1].h = 240
	Config.camera[1].focal_length = 320
end

return Config
