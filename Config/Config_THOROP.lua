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

Config.sensors = {
  ft = true,
  head_camera = true,
  chest_lidar = true,
  head_lidar = false,
  fsr = false,
}

Config.use_localhost = false
if IS_WEBOTS then
  -- Tune which wizards to run in webots
  Config.wizards = {}
  Config.wizards.mesh = 'mesh_wizard'
  Config.wizards.world = 'world_wizard'
  Config.wizards.camera = 'camera_wizard'
  Config.wizards.test = 'post_mesh_wizard'
  --Config.wizards.slam = 'slam_wizard'
  -- Adjust the tiemsteps if desired
  --Config.camera_timestep = 33
--  Config.lidar_timestep = 200 --slower

  Config.use_localhost = true
end

-- Printing of debug messages
Config.debug = {
	webots_wizard = false,	
  obstacle = false,
  follow = false,	
  approach = false,
  planning = false,
  goalpost = false,
  world = false,
--  world = true,
}

-- Monitor and logging
Config.enable_monitor = true
Config.enable_log = false
Config.use_log = false
Config.torque_legs = true

--Default motion libraries
Config.libs={
  ArmLib = 'DRCTrials',
  MotionLib = 'RoboCup',  
  World = 'DRCNew'  
}


--SJ: now we can choose which config, fsm and mid-level libraries to use

--Robocup 
--[[
Config.libs.World = 'RoboCup'
local exo = {
  'Robot','Walk','Net','Manipulation',
 -- 'FSM_RoboCup','World_RoboCup','Vision_RoboCup'
 'FSM_KickDemo','World_RoboCup','Vision_RoboCup' 
}
Config.testfile = 'test_robocup'
Config.sensors.chest_lidar = false
Config.wizards.test = nil
Config.wizards.mesh = nil
--]]


--[[
--DRC Trials
local exo = {'Robot','Walk','Net','Manipulation',
'FSM_DRCTrials','World_RoboCup','Vision_DRCTrials'
}
Config.testfile = 'test_robocup'
--]]


----[[
--DRC Site visit 2014
Config.libs.World = 'SiteVisit'   
Config.wizards.mesh = 'mesh_wizard_sitevisit'
----[[
Config.sensors.chest_lidar = false
Config.wizards.test = nil
Config.wizards.mesh = nil
--]]
local exo = {'Robot','Walk','Net','Manipulation',
'FSM_SiteVisit','World_DRCTrials','Vision_DRCTrials'
}
Config.testfile = 'test_sitevisit'
--]]




--[[
-- Teach robot to go up steps
Config.libs.MotionLib = 'Teach'}
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

Config.use_gps_pose = false
Config.use_imu_yaw = true

return Config
