local Config = {}

local vector = require('vector')
local unix = require('unix')
local util = require'util'

Config.platform = {} 
Config.platform.name = 'THOROP'

-- Parameters Files
platform = {};
platform.name = 'WebotsTHOROP'
params = {}
params.name = {"Walk", "Kick", "Vision", "FSM"}
---Location Specific Camera Parameters
params.Camera = "Grasp"
Config = util.LoadConfig(params, platform,Config)

-- Device Interface Libraries
Config.dev = {}
Config.dev.body = 'WebotsTHOROPBody2'
Config.dev.kinematics = 'THOROPKinematics'
Config.dev.ip_wired = '192.168.123.255' 
Config.dev.ip_wired_port = 111111
Config.dev.ip_wireless = '192.168.1.255' --Our Router
Config.dev.ip_wireless_port = 54321
Config.dev.game_control='OPGameControl'
Config.dev.team='TeamNSL'
--Config.dev.walk='BetterWalk'
Config.dev.walk='EvenBetterWalk'
Config.dev.kick = 'NewNewKick'
Config.dev.walk = 'HumbleWalk';
Config.dev.crawl = 'ScrambleCrawl';
--dev.largestep = 'ZMPStep';
Config.dev.largestep = 'ZMPStepStair';
Config.dev.gender = 1 -- 1 for boy and 0 for girl 

-- keyframe files
Config.km = {};
Config.km.standup_front = 'km_Charli_StandupFromFront.lua';
Config.km.standup_back = 'km_Charli_StandupFromBack.lua';

return Config
