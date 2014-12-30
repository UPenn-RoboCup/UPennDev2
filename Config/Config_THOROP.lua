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

Config.use_localhost = false
Config.sensors = {
  ft = true,
  head_camera = false, --true,
  chest_lidar = true, --false,
  head_lidar = false,
  kinect = false,
  fsr = false,
}

Config.wizards = {}


-- Printing of debug messages
Config.debug = {
	webots_wizard = false,	
  obstacle = false,
  follow = false,	
  approach = false,
  planning = false,
  goalpost = false,
  world = false,
  feedback = false,
  detect = true,
}


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


--Choose which config, fsm and mid-level libraries to use


-- Karen perception
Config.libs.World = 'SiteVisit'   
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_SiteVisit','World_DRCTrials','Vision_DRCFinal' 
}
Config.testfile = 'test_sitevisit'




-- Robocup 
--[[
Config.libs.World = 'RoboCup'
local exo = {
  'Robot','Walk','Net','Manipulation',
 'FSM_RoboCup','World_RoboCup','Vision_RoboCup' 
}
Config.testfile = 'test_robocup'
Config.sensors.chest_lidar = false
Config.wizards.test = nil
Config.wizards.mesh = nil
--]]



-- Teach robot to go up steps
--[[
Config.libs.MotionLib = 'Teach'
-- Precedence in loading, for overrides!
local exo = {
	'Robot', 'Walk', 'Net', 'FSM_Teach'
}
Config.testfile = 'test_teach'
Config.wizards = {}
Config.sensors = {ft = true}
if IS_WEBOTS then
  Config.testfile = 'test_sitevisit'
end
--]]


-- Steve Manipulation and Locomotion
--[[
Config.libs.MotionLib = 'Teach'
Config.libs.ArmLib = 'Teach'
Config.libs.World = 'Teach'
-- Precedence in loading, for overrides!
local exo = {
	'Robot', 'Walk', 'Net',
	'FSM_Teach', 'Arm_Teach', 'Vision_Teach' --, 'World_Teach'
}
if IS_WEBOTS then
  Config.testfile = 'test_teleop'
  Config.sensors.kinect = true
  Config.sensors.chest_lidar = true
	Config.wizards.kinect = 'kinect2_wizard'
  Config.wizards.mesh = 'mesh_wizard'
end
--]]


-- DRC Site visit 2014
--[[
Config.libs.World = 'SiteVisit'   
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_SiteVisit','World_DRCTrials','Vision_RoboCup' 
}
if IS_WEBOTS then
	Config.sensors.head_camera = true
  Config.testfile = 'test_sitevisit'
end
--]]



-- Remote Control for testing with Marcell
--[[
local exo = {
	'Robot', 'Walk', 'Net', 'FSM_Remote'
}
Config.libs.MotionLib = 'RoboCup'
Config.wizards = {
  feedback = 'feedback_wizard',
  --remote = 'remote_wizard',
}
Config.testfile = 'test_remote'
Config.sensors.chest_lidar = false
--]]


-- Webots specified
if IS_WEBOTS then
  Config.use_localhost = true
  -- Default Webots sensors
  Config.sensors = {
    ft = true,
    head_camera = false,
    chest_lidar = true,
    head_lidar = false,
    kinect = false,
  }
  -- Tune which wizards to run in webots
  Config.wizards = {
    feedback = 'feedback_wizard',
    mesh = Config.sensors.chest_lidar and 'mesh_wizard',
    world = false, --'world_wizard',
    camera = Config.sensors.head_camera and 'camera_wizard',
    kinect = false,  --'kinect2_wizard',
    detect = 'detect_wizard',
    remote = false, -- 'remote_wizard',
  }  
  -- Adjust the tiemsteps if desired
  -- Config.camera_timestep = 33
  -- Config.lidar_timestep = 200 --slower
  Config.kinect_timestep = 300  -- slower
  
  Config.testfile = 'test_sitevisit'
end


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
Config.use_single_scan = false

return Config
