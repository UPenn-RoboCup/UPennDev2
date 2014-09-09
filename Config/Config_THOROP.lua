Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'THOROP'
Config.nJoint = 35
-- Dummy arms are the two MX-106R motors per arm
Config.USE_DUMMY_ARMS = false

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {
	body         = 'THOROPBody',
	gender       = 'boy',
}

--SJ: now we can choose which config, fsm and mid-level libraries to use

--Robocup 
--[[
Config.libs = {
  ArmLib = 'DRCTrials',
  MotionLib = 'RoboCup',
  World = 'RoboCup'
}
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_RoboCup','World_RoboCup','Vision_RoboCup'
}
--]]

--[[
--DRC Trials
Config.libs = {
  ArmLib = 'DRCTrials',
  MotionLib = 'RoboCup',
  World = 'DRCNew'
}
local exo = {'Robot','Walk','Net','Manipulation',
'FSM_DRCTrials','World_RoboCup','Vision_DRCTrials'
}
--]]


----[[
--DRC Site visit 2014
Config.libs = {
  ArmLib = 'DRCTrials',
  MotionLib = 'RoboCup',
  World = 'SiteVisit'   
}
local exo = {'Robot','Walk','Net','Manipulation',
'FSM_SiteVisit','World_DRCTrials','Vision_DRCTrials'
}
--]]





--Add path to selected librares
for i,sm in pairs(Config.libs) do
  local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
  package.path = table.concat(pname)
end

-- Printing of debug messages
Config.debug = {
	webots_wizard=false,	
  -- obstacle = true,
  follow = false,	
  --approach = true,
  --planning = true,
  --goalpost = true,
}

--Config.use_localhost = false
Config.use_localhost = true


-- Monitor and logging
Config.enable_monitor = true
Config.enable_log = false
Config.use_log = false

-------------
-- Complementary Configs --
---------------------------
Config.torque_legs = true

-- Load each exogenous Config file
for _,v in ipairs(exo) do
	--[[
  --local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  local fname = {'Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  dofile(table.concat(fname))
	--]]
	----[[
	local fname = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v}  
  require(table.concat(fname))
	--]]
end

Config.supportY_preview = -0.02
Config.supportY_preview2 = -0.01

return Config
