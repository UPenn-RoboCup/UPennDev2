assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.human = {}
Config.human.touch = {
  ch = 'touche',
  reliable = 55588,
}

return Config
