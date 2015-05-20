IS_STEVE = true
--IS_STEVE = false

IS_COMPETING = false

-- Global Config
Config = {
	PLATFORM_NAME = 'THOROP2',
	demo = false,
}

local exo = {
	'Walk','Net','FSM','World','Vision',
	'Robot_Chipette',
	(IS_STEVE and 'Arm_Steve' or 'Arm_DRCFinal'),
}

Config.toeheel_lift = true
Config.enable_touchdown = false
Config.raise_body = true
Config.waist_testing = true
Config.use_jacobian_arm_planning = true
Config.piecewise_step = true
Config.enable_jacobian_test = false
--birdwalk TODO : IMU and FT values swap
--Config.birdwalk = 1 --testing birdwalk

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
	--Config.testfile = 'test_balance'
  --Config.testfile = 'test_testbed'
 -- Config.testfile = 'test_walkstuff'
   Config.testfile = 'test_teleop2'
	--
  Config.sensors = {
		ft = true,
		feedback = 'feedback_wizard',
		--slam = true,
    --head_camera = 'camera_wizard',
    --chest_lidar = true,
    --head_lidar = true,
    --kinect = 'kinect2_wizard',
		mesh = 'mesh_wizard',
	 	world = 'world_wizard',
  }
  -- Adjust the timesteps if desired
  -- Config.camera_timestep = 33
  -- Config.lidar_timestep = 200 --slower
  -- Config.kinect_timestep = 30
end

-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/THOROP0/?.lua;', package.path}
package.path = table.concat(pname)
if Config.demo then table.insert(exo, 'Demo') end
for _,v in ipairs(exo) do
	local fname = {'Config_THOROP0_', v}
	local filename = table.concat(fname)
  assert(pcall(require, filename))
end

-- Custom motion libraries
for i,sm in pairs(Config.fsm.libraries) do
	local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
	package.path = table.concat(pname)
end

-- Finite state machine paths
for sm, en in pairs(Config.fsm.enabled) do
	if en then
		local selected = Config.fsm.select[sm]
		if selected then
			local pname = {HOME, '/Player/', sm, 'FSM/', selected, '/?.lua;', package.path}
			package.path = table.concat(pname)
		else --default fsm
			local pname = {HOME, '/Player/', sm, 'FSM/', '?.lua;', package.path}
			package.path = table.concat(pname)
		end
	end
end

return Config
