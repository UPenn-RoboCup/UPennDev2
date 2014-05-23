assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.human = {}
Config.human.touch = {
  ch = 'touch',
  reliable = 55588,
  bbox_ch = 'bbox'
}

return Config
