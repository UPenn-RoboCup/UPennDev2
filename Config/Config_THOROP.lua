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
Config.testfile = 'test_robocup'
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
Config.testfile = 'test_robocup'
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
Config.testfile = 'test_sitevisit'
--]]


Config.camera_timestep = 33
Config.lidar_timestep = 200 --slower

--[[
-- Teach robot to go up steps
Config.libs = {
  MotionLib = 'RoboCup',
}
-- Precedence in loading, for overrides!
local exo = {
	'Robot', 'Walk', 'Net', 'FSM_Teach'
}
Config.testfile = 'test_teach'
--]]


--Add path to selected librares
for i,sm in pairs(Config.libs) do
  local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
  package.path = table.concat(pname)
end

-- Printing of debug messages
Config.debug = {
	webots_wizard=false,	
  obstacle = false,
  follow = false,	
  approach = false,
  planning = false,
  goalpost = false,
}

if IS_WEBOTS then
  -- Tune which wizards to run in webots
  Config.wizards = {}
  --Config.wizards.mesh = 'mesh_wizard'
  Config.wizards.mesh = 'mesh_wizard_sitevisit'
  Config.wizards.world = 'world_wizard'
  Config.wizards.camera = 'camera_wizard'
  Config.wizards.test = 'post_mesh_wizard'
  --Config.wizards.slam = 'slam_wizard'
end

--Config.use_localhost = false
Config.use_localhost = true

-- Monitor and logging
Config.enable_monitor = true
Config.enable_log = false
Config.use_log = false
Config.torque_legs = true

-- Complementary Configs --
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

return Config
