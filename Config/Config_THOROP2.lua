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


Config.arm_init_timeout = true
Config.use_imu_yaw = true


-- Tune for Webots
if IS_WEBOTS then

	--for SJ's testing in webots
		Config.testfile = 'test_walk_robocup'
		Config.piecewise_step = true
	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      vision = 'vision_wizard',
      world = 'world_wizard',

			--feedback = 'feedback_wizard',
	  }
	if IS_STEVE then		
		Config.use_gps_pose = true
		Config.use_gps_vision = true
	else
		
		Config.use_gps_pose = false
		Config.use_gps_vision = false
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
