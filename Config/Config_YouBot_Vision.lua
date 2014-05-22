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
  params = {

  },
})

return Config
