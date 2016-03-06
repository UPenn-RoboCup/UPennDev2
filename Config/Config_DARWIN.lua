--Now the Config file is entirely identical over two robots (using hostname)
------------WE USE THOR1 CONFIG FOR ROBOCUP ------------------------


IS_STEVE = true
IS_COMPETING = false

if HOSTNAME=="thor-P770ZM" or HOSTNAME=="asus"then	IS_STEVE = false end
-- Global Config
Config = {PLATFORM_NAME = 'DARWIN',demo = false,}
local exo = {'Walk','Net','FSM','World','Vision','Robot', 'Arm', 'Kick'}
print("PLATFORM NAME:"..Config.PLATFORM_NAME)
--Config = {PLATFORM_NAME = 'THOROP2',demo = false,}
--exo = {'Walk','Net','FSM','World','Vision','Robot_Chip', 'Arm'}

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
  armplan = false,
  walk = false,


  --goalpost = true,
}

Config.arm_init_timeout = true
Config.use_imu_yaw = false --use odometry for yaw

-----------------------------------

-- Tune for Webots
if IS_WEBOTS then

	--for SJ's testing in webots
		--Config.testfile = 'test_testbed'
		--Config.testfile = 'test_robocup'
		Config.testfile = 'test_robocup'


--		Config.testfile = 'test_testbed'
	Config.testfile = 'test_walk_robocup'
	Config.testfile = 'test_stair'

		Config.piecewise_step = true
--[[		
	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      vision = 'vision_wizard',
      world = 'world_wizard',

			--feedback = 'feedback_wizard',
	  }
--]]

	  Config.sensors = {
	  }




	if IS_STEVE then
    print('!!!!IS_STEVE!!!!')
		Config.use_gps_pose = false
		Config.use_gps_vision = false

	else
		Config.use_gps_pose = false
		Config.use_gps_vision = false

	--Config.use_gps_pose = true
--		Config.use_gps_vision = true


	end
end


-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/DARWIN/?.lua;', package.path}
package.path = table.concat(pname)
if Config.demo then table.insert(exo, 'Demo') end
for _,v in ipairs(exo) do
	--print('Loading', v)
	local fname = {'Config_DARWIN_', v}
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
--[[

--MOVING ZMP
Config.walk.heeltoe_vel_min = 0.19 --MIN speed for heeltoe walk
Config.walk.zmpHeel = -0.05
Config.walk.zmpToe = 0.05
Config.use_heeltoe_walk=true
Config.walk.zmpHeel,Config.walk.zmpToe = -0.0,0.10


Config.walk.heel_angle = 5*DEG_TO_RAD
Config.walk.toe_angle = 5*DEG_TO_RAD
Config.walk.heel_angle = 10*DEG_TO_RAD
Config.walk.toe_angle = 10*DEG_TO_RAD


--No heeltoe, no moving zmp
Config.use_heeltoe_walk=false
Config.walk.zmpHeel,Config.walk.zmpToe = 0,0
Config.walk.heel_angle = 0*DEG_TO_RAD
Config.walk.toe_angle = 0*DEG_TO_RAD
--]]

return Config
