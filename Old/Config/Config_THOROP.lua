-- Global Config
Config = {}

-- General parameters
Config.PLATFORM_NAME = 'THOROP'
Config.nJoint = 35
Config.use_localhost = false
-- Dummy arms are the two MX-106R motors per arm
Config.USE_DUMMY_ARMS = false
Config.dev = {
	body         = 'THOROPBody',
	gender       = 'boy',
}

-- Default motion libraries
Config.libs = {
  ArmLib = 'DRCTrials',
  MotionLib = 'RoboCup',  
  World = 'DRCNew'
}

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
}

-- Tune for Webots
if IS_WEBOTS then
  Config.use_localhost = true
  Config.wizards = {
    feedback = 'feedback_wizard',
    mesh = false, --'mesh_wizard',
    world = false, --'world_wizard',
    camera = false, --'camera_wizard'
    kinect = false, --'kinect2_wizard',
  }
  -- Default Webots sensors
  Config.sensors = {
    ft = true,
    head_camera = false,
    chest_lidar = false,
    head_lidar = false,
    kinect = false,
  }
  -- Adjust the tiemsteps if desired
  -- Config.camera_timestep = 33
  -- Config.lidar_timestep = 200 --slower
  -- Config.kinect_timestep = 30
  Config.use_gps_pose = false
end

-- TODO: Get rid of these
Config.enable_monitor = true
Config.torque_legs = true

----------------------------------
-- Application specific Configs --
----------------------------------

-- DRC Final setup: testing new walk controller
----[[
Config.libs.World = 'SiteVisit'   
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_DRCFinal','World_DRCTFinal','Vision_DRCFinal' 
}
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

-- Robocup
--[[
Config.libs.World = 'RoboCup'
local exo = {
  'Robot','Walk','Net','Manipulation',
 'FSM_RoboCup','World_RoboCup','Vision_RoboCup'
 -- 'FSM_KickDemo','World_RoboCup','Vision_RoboCup'
}
Config.testfile = 'test_robocup'
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
if IS_WEBOTS then
  Config.wizards = {
    feedback = 'feedback_wizard',
    --remote = 'remote_wizard',
  }
  Config.testfile = 'test_remote'
end
--]]








-----------------------------------
-- Load Paths and Configurations --
-----------------------------------

-- Custom motion libraries
for i,sm in pairs(Config.libs) do
  local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
  package.path = table.concat(pname)
end

-- Custom Config files
for _,v in ipairs(exo) do
	local fname = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v}  
  require(table.concat(fname))
end

-- Finite state machine paths
for _,sm in ipairs(Config.fsm.enabled) do
  local selected = Config.fsm.select[sm]
  if selected then
    local pname = {HOME, '/Player/', sm, 'FSM/', selected, '/?.lua;', package.path}
    package.path = table.concat(pname)
  else --default fsm
    local pname = {HOME, '/Player/', sm, 'FSM/', '?.lua;', package.path}
    package.path = table.concat(pname)
  end  
end

return Config
