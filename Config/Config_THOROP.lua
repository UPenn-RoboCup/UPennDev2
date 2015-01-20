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
local exo = {
  'Robot','Walk','Net','Manipulation',
  'FSM_DRCFinal','World_DRCFinal','Vision_DRCFinal' 
}
if IS_WEBOTS then
--  Config.testfile = 'test_sitevisit'
  Config.testfile = 'test_balance'
end

-----------------------------------
-- Load Paths and Configurations --
-----------------------------------

-- Load custom Config files first 
for _,v in ipairs(exo) do
  local fname = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v} 
  require(table.concat(fname))
end


-- Load custom libraries
for i,sm in pairs(Config.fsm.libraries) do
  local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
  package.path = table.concat(pname)
end

-- Load custom FSMs
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
