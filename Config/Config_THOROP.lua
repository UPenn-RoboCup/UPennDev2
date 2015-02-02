-- Global Config
Config = {}

-- TODO: Get rid of these
Config.enable_monitor = true
Config.torque_legs = true

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
  -- Default Webots sensors
  Config.sensors = {
		ft = true,
		feedback = 'feedback_wizard',
    head_camera = 'camera_wizard',
    chest_lidar = 'mesh_wizard',
    head_lidar = 'slam_wizard',
    kinect = 'kinect2_wizard',
		world = 'world_wizard',
  }
  -- Adjust the tiemsteps if desired
  -- Config.camera_timestep = 33
  -- Config.lidar_timestep = 200 --slower
  -- Config.kinect_timestep = 30
  Config.use_gps_pose = false
end

----------------------------------
-- Application specific Configs --
----------------------------------

-- DRC Final setup: testing new walk controller
----[[
Config.testfile = 'test_balance'
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_DRCFinal','World_DRCFinal','Vision_DRCFinal'
}
--]]


-- Steve Manipulation and Locomotion
--[[
Config.testfile = 'test_teleop'
local exo = {
	'Robot', 'Walk', 'Net',
	'FSM_Teach', 'Arm_Teach', 'Vision_Teach' --, 'World_Teach'
}
if IS_WEBOTS then
  Config.kinect_timestep = 50
end
--]]

-----------------------------------
-- Load Paths and Configurations --
-----------------------------------

-- Custom Config files
for _,v in ipairs(exo) do
	local fname = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v}
  require(table.concat(fname))
end

-- Custom motion libraries
for i,sm in pairs(Config.libraries) do
	local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
	package.path = table.concat(pname)
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
