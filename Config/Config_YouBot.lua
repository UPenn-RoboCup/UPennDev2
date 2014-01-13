local Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'YouBot'

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
Config.fsm.enabled = {}

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}

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
-- Head
Config.camera.head = {}
Config.camera.head.device = '/dev/video0'
Config.camera.head.format = 'yuyv'
Config.camera.head.resolution = {640,360}
--Config.camera.head.resolution = {320,180}
Config.camera.head.fps = 10

-- Forehead (wide angle)
Config.camera.forehead = {}
Config.camera.forehead.device = '/dev/video1'
Config.camera.forehead.format = 'yuyv'
--Config.camera.forehead.format = 'mjpeg'
--Config.camera.forehead.resolution = {160,120}
Config.camera.forehead.resolution = {320,240}
Config.camera.forehead.fps = 10

--[[
Config.camera.forehead2 = {}
Config.camera.forehead2.device = '/dev/video2'
Config.camera.forehead2.format = 'yuyv'
--Config.camera.forehead2.format = 'mjpeg'
Config.camera.forehead2.resolution = {320,240}
Config.camera.forehead2.fps = 5
--]]

return Config
