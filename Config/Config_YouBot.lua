local Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'YouBot'
Config.nJoint = 5

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.body         = Config.PLATFORM_NAME..'Body'

--------------------
-- State Machines --
--------------------
Config.fsm = {}
-- Which FSMs should be enabled?
Config.fsm.enabled = {'BodyFSM'}
if HOME then
	-- Add all FSM directories that are in Player
	for _,sm in ipairs(Config.fsm.enabled) do
		package.path = HOME..'/Player/'..sm..'/?.lua;'..package.path
	end
end

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}
exo.Net = 'Net'

for k,v in pairs(exo) do
	-- TODO SJ: Now we just put all config files in /Config folder
	--local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
	local exo_name = '/Config_'..Config.PLATFORM_NAME..'_'..v
	local exo_config = require(exo_name)
	for kk,vv in pairs(exo_config) do Config[kk] = vv end
end

---------------
-- Keyframes --
---------------
Config.km = {}
Config.km.standup_front = 'km_Charli_StandupFromFront.lua'
Config.km.standup_back  = 'km_Charli_StandupFromBack.lua'

-------------
-- Cameras --
-------------
Config.camera = {}

table.insert(Config.camera,
{
	name = 'head',
	dev = '/dev/video1',
	fmt = 'yuyv',
	width = 320,
	height = 240,
	fps = 30,
	focal_length = 184.75, -- horizontal guess...
	params = {

	},
})

return Config
