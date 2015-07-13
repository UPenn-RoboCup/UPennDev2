--Now the Config file is entirely identical over two robots (using hostname)
IS_STEVE = true
IS_COMPETING = false

if HOSTNAME=="thor-P770ZM" or HOSTNAME=="asus"then	IS_STEVE = false end
-- Global Config
--Config = {PLATFORM_NAME = 'THOROP1',demo = false,}
--local exo = {'Walk','Net','FSM','World','Vision','Robot_Dale', 'Arm'}

Config = {PLATFORM_NAME = 'THOROP2',demo = false,}
exo = {'Walk','Net','FSM','World','Vision','Robot_Chip', 'Arm','Kick'}


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
  armplan = true,
}


--Config.birdwalk = 1
Config.raise_body = true
Config.use_exact_tZMP = true
Config.use_heeltoe_walk = true
Config.heeltoe_angle = 0*DEG_TO_RAD
Config.walktraj={}
Config.walktraj.hybridwalk = "foot_trajectory_softfast"
Config.walktraj.hybridwalk = "foot_trajectory_base"
Config.variable_tstep = true
Config.variable_support = true
Config.arm_init_timeout = true
Config.use_imu_yaw = true

Config.estop_mode = 0 --don't do anything!
Config.estop_mode = 1 --untorque all the servos 
--Config.estop_mode = 2 --make the robot sit down
Config.auto_restart = true

--Config.hybrid_approach = true

Config.roll_adaptation_max = 3.5*DEG_TO_RAD
Config.pitch_adaptation_max = 2*DEG_TO_RAD
--Config.pitch_adaptation_max = 0*DEG_TO_RAD --disabled

Config.pitch_threshold = 1*DEG_TO_RAD
Config.pitch_adaptation_max = 3*DEG_TO_RAD --disabled

--NO adaptation!
Config.adapt_surface_angle =false
Config.roll_adaptation_max = 0*DEG_TO_RAD
Config.pitch_adaptation_max = 0*DEG_TO_RAD


Config.comX_bias = 0


-- Tune for Webots
if IS_WEBOTS then
	if IS_STEVE then
		Config.testfile = 'test_teleop'
		Config.debug.armplan = true

	  Config.sensors = {
			--ft = true,
			--feedback = 'feedback_wizard',
		--slam = 'slam_wizard',
    --head_camera = 'camera_wizard',
    --chest_lidar = true,
    --head_lidar = true,
    --kinect = 'kinect2_wizard',
			--mesh = 'mesh_wizard',
		 	--world = 'world_wizard',
	  }
	else
		--Config.testfile = 'test_testbed'		
--		Config.testfile = 'test_walkstuff'		

		--Config.testfile = 'test_testbed'		
		Config.testfile = 'test_terrain'		

		Config.debug.armplan = false		
		Config.use_jacobian_arm_planning = true
		Config.enable_jacobian_test = false
		--Config.enable_jacobian_test = true
		Config.enable_touchdown = false
	  Config.sensors = {
			ft = true,
			feedback = 'feedback_wizard',
		 	world = 'world_wizard',
	  }

		Config.use_imu_yaw = false --use imu yaw only for single approach
	end
end

-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/THOROP0/?.lua;', package.path}
package.path = table.concat(pname)
if Config.demo then table.insert(exo, 'Demo') end
for _,v in ipairs(exo) do
	--print('Loading', v)
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




if IS_WEBOTS then
	Config.world.odomDrift = 0
else
	Config.world.odomDrift = -0.0001
end


--robot drifts backwards
--Config.world.odomScale = {0.8,1,1} -- For now IMU not in use
Config.world.use_imu_yaw = true


Config.slowstep_duration =2.5
Config.supportYSS = -0.03
Config.walk.stepHeightSlow = 0.02

return Config
