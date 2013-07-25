local Config = {}

local vector = require('vector')
local unix = require('unix')

Config.platform = {} 
Config.platform.name = 'THOROP'

-- Parameters Files
params = {}
params.name = {"Robot", "Walk", "World", "Kick", "Vision", "FSM", "Camera"}
---Location Specific Camera Parameters
params.Camera = "Grasp"

-- Device Interface Libraries
Config.dev = {}
Config.dev.body = 'ThorOPBody' 
Config.dev.kinematics = 'ThorOPKinematics'
Config.dev.ip_wired = '192.168.123.255' 
Config.dev.ip_wired_port = 111111
Config.dev.ip_wireless = '192.168.1.255' --Our Router
Config.dev.ip_wireless_port = 54321
Config.dev.game_control='OPGameControl'
Config.dev.team='TeamNSL'
--Config.dev.walk='BetterWalk'
Config.dev.walk='EvenBetterWalk'
Config.dev.kick = 'NewNewKick'
Config.dev.gender = 1 -- 1 for boy and 0 for girl 

return Config