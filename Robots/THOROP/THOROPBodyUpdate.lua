--------------------------------
-- Body abstraction for THOR-OP
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
--------------------------------
assert(ffi, 'Need LuaJIT to run. Lua support in the future')
-- Shared memory for the joints
require'dcm'
-- Shared memory for world
require'wcm'

-- Utilities
local unix   = require'unix'
local vector = require'vector'
local util   = require'util'
local si     = require'simple_ipc'
local Kinematics = require'THOROPKinematics'

local Body = {}
local dev_chs, dcm_chs, body_chs, body_poll = {}, {
	si.new_publisher'body!'
	}, {}
local get_time = unix.time

-- TODO: Body or Config?
-- Shared memory layout
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexLArm = 3   --LArm: 3 4 5 6 7 8 9
local nJointLArm = 7
local indexLLeg = 10  --LLeg: 10 11 12 13 14 15
local nJointLLeg = 6
local indexRLeg = 16  --RLeg: 16 17 18 19 20 21
local nJointRLeg = 6
local indexRArm = 22  --RArm: 22 23 24 25 26 27 28
local nJointRArm = 7
local indexWaist = 29  --Waist: 29 30
local nJointWaist = 2
-- 4 Servos for gripping
-- 1 servo for lidar panning
local indexLGrip = 31
local nJointLGrip = 2
local indexRGrip = 33
local nJointRGrip = 2
-- One motor for lidar panning
local indexLidar = 35
local nJointLidar = 1
local nJoint = 35

local parts = {
	Head=vector.count(indexHead,nJointHead),
	LArm=vector.count(indexLArm,nJointLArm),
	LLeg=vector.count(indexLLeg,nJointLLeg),
	RLeg=vector.count(indexRLeg,nJointRLeg),
	RArm=vector.count(indexRArm,nJointRArm),
	Waist=vector.count(indexWaist,nJointWaist),
	LGrip=vector.count(indexLGrip,nJointLGrip),
  RGrip=vector.count(indexRGrip,nJointRGrip),
  Lidar=vector.count(indexLidar,nJointLidar)
}

------------------
-- Body sensors --
------------------
for sensor, ptr in pairs(dcm.sensorPtr) do
	local cur = dcm['get_sensor_'..sensor]()
	local n_el = type(cur)=='table' and #cur or 1
	local get = function(idx1, idx2)
		-- For cdata, use -1
		return vector.slice(ptr, (idx1 or 1)-1, (idx2 or n_el)-1)
	end
  Body['get_'..sensor] = get
  -- Anthropomorphic access to dcm
	-- TODO: get_lleg_rpy is illegal, for instance
  for part, jlist in pairs(parts) do
		-- For cdata, use -1
    local idx1, idx2 = jlist[1], jlist[#jlist]
    Body['get_'..part:lower()..'_'..sensor] = function(idx)
      if idx then return get(jlist[idx]) else return get(idx1, idx2) end
    end -- Get
  end
	-- End anthropomorphic
end

--------------------
-- Body actuators --
--------------------
for actuator, ptr in pairs(dcm.actuatorPtr) do
	-- Only command_position is constantly synced
	-- Other commands need to be specially sent to the Body
	local not_synced = actuator~='command_position'
	local function set(val, idx1, idx2)
		-- cdata is -1
		if idx2 then
			if type(val)=='number' then
				for i=idx1, idx2 do ptr[i - 1] = val end
			else
				local idx
				for i,v in ipairs(val) do
					idx = idx1 + i - 1
					if idx>idx2 then break else ptr[idx - 1] = v end
				end
			end
		elseif idx1 then
			if type(val)=='number' then
				ptr[idx1 - 1] = val
			else
				for i,v in ipairs(val) do ptr[idx1 + i - 2] = v end
			end
		else
			-- No index means set all actuators... Uncommon
			if type(val)=='number' then
				for i=0, nJoint-1 do ptr[i] = val end
			else
				for i, v in ipairs(val) do ptr[i - 1] = v end
			end
		end
		-- Send msg to the dcm, just string of the id
		if not_synced then
			for _, ch in ipairs(dcm_chs) do
        --print('SET', ch, actuator)
        ch:send(actuator)
      end
		end
	end
	local function get(idx1, idx2)
		idx1 = idx1 or 1
		idx2 = idx2 or nJoint
		-- For cdata, use -1
		return vector.slice(ptr, idx1 - 1, idx2 - 1)
	end
	-- Export
  Body['set_'..actuator] = set
  Body['get_'..actuator] = get
  --------------------------------
  -- Anthropomorphic access to dcm
  -- TODO: Do not use string concatenation to call the get/set methods of Body
  for part, jlist in pairs(parts) do
		local idx1, idx2, idx = jlist[1], jlist[#jlist], nil
		Body['get_'..part:lower()..'_'..actuator] = function(idx)
			if idx then return get(jlist[idx]) else return get(idx1, idx2) end
		end
		Body['set_'..part:lower()..'_'..actuator] = function(val, i)
			idx = jlist[i]
			if idx then return set(val, idx) else return set(val, idx1, idx2) end
		end
  end
	-- End anthropomorphic
end

-- TODO: Set if this is the right approach...
Body.set_rgrip_percent = function( percent, is_torque )
  -- Convex combo
  percent = math.min(math.max(percent,0),1)
  --
  local thumb = indexRGrip
  local radian = (1-percent)*servo.min_rad[thumb] + percent*servo.max_rad[thumb]
  dcm.actuatorPtr.command_position[thumb] = radian
  dcm.writePtr.command_position[thumb] = 1
  -- Set the command_torque
  dcm.gripperPtr.torque_mode[3] = 0
  -- Set the command_torque to zero
  dcm.gripperPtr.command_torque[3] = 0
end
-- For torque control (no reading from the motor just yet)
Body.set_rgrip_command_torque = function(val)
  -- Set the command_torque
  dcm.gripperPtr.command_torque[3] = -1*val
  -- Set the command_torque
  dcm.gripperPtr.torque_mode[3] = 1
end
-- For torque control (no reading from the motor just yet)
Body.get_rgrip_command_torque_step = function()
  local val = dcm.gripperPtr.command_torque[3]
  -- Not too large/small
  val = util.procFunc(val,0,1023)
  if val<0 then val=1024-val end
  -- Return the value and the mode
  return val, dcm.gripperPtr.torque_mode[3]
end
Body.set_rtrigger_percent = function( percent, is_torque )
  local thumb = indexRGrip+1
  percent = math.min(math.max(percent,0),1)
  -- Convex combo
  local radian = (1-percent)*servo.min_rad[thumb] + percent*servo.max_rad[thumb]
  dcm.actuatorPtr.command_position[thumb] = radian
  dcm.writePtr.command_position[thumb] = 1
  -- Set the command_torque to position
  dcm.gripperPtr.torque_mode[4] = 0
  -- Set the command_torque to zero
  dcm.gripperPtr.command_torque[4] = 0
end
-- For torque control (no reading from the motor just yet)
Body.set_rtrigger_command_torque = function(val)
  -- Set the command_torque
  dcm.gripperPtr.command_torque[4] = val
  -- Set the command_torque
  dcm.gripperPtr.torque_mode[4] = 1
end
-- For torque control (no reading from the motor just yet)
Body.get_rtrigger_command_torque_step = function()
  local val = dcm.gripperPtr.command_torque[4]
  -- Not too large/small
  val = util.procFunc(val,0,1023)
  if val<0 then val=1024-val end
  -- Return the value and the mode
  return val, dcm.gripperPtr.torque_mode[4]
end

-- left --

-- For position control
Body.set_lgrip_percent = function( percent, is_torque )
  local thumb = indexLGrip
  percent = math.min(math.max(percent,0),1)
  -- Convex combo
  local radian = (1-percent)*servo.min_rad[thumb] + percent*servo.max_rad[thumb]
  dcm.actuatorPtr.command_position[thumb] = radian
  dcm.writePtr.command_position[thumb] = 1
  -- Set the command_torque to position
  dcm.gripperPtr.torque_mode[1] = 0
  -- Set the command_torque to zero
  dcm.gripperPtr.command_torque[1] = 0
end
-- For torque control (no reading from the motor just yet)
Body.set_lgrip_command_torque = function(val)
  -- Set the command_torque
  dcm.gripperPtr.command_torque[1] = val
  -- Set the command_torque
  dcm.gripperPtr.torque_mode[1] = 1
end
-- For torque control (no reading from the motor just yet)
Body.get_lgrip_command_torque_step = function()
  local val = dcm.gripperPtr.command_torque[1]
  -- Not too large/small
  val = util.procFunc(val,0,1023)
  if val<0 then val=1024-val end
  -- Return the value and the mode
  return val, dcm.gripperPtr.torque_mode[1]
end
Body.set_ltrigger_percent = function( percent, is_torque )
  local thumb = indexLGrip+1
  percent = math.min(math.max(percent,0),1)
  -- Convex combo
  local radian = (1-percent)*servo.min_rad[thumb] + percent*servo.max_rad[thumb]
  dcm.actuatorPtr.command_position[thumb] = radian
  dcm.writePtr.command_position[thumb] = 1
  -- Set the command_torque to position
  dcm.gripperPtr.torque_mode[2] = 0
  -- Set the command_torque to zero
  dcm.gripperPtr.command_torque[2] = 0
end
-- For torque control (no reading from the motor just yet)
Body.set_ltrigger_command_torque = function(val)
--print('val!',val)
  -- Set the command_torque
  dcm.gripperPtr.command_torque[2] = -1*val
  -- Set the command_torque
  dcm.gripperPtr.torque_mode[2] = 1
end
-- For torque control (no reading from the motor just yet)
Body.get_ltrigger_command_torque_step = function()
  local val = dcm.gripperPtr.command_torque[2]
  -- Not too large/small
  val = util.procFunc(val,0,1023)
  if val<0 then val=1024-val end
  -- Return the value and the mode
  return val, dcm.gripperPtr.torque_mode[2]
end

-- Check the error from a desired transform tr
-- to a forwards kinematics of in IK solution q
local function check_ik_error( tr, tr_check, pos_tol, ang_tol )

  -- Tolerate a 1mm error in distance
  pos_tol = pos_tol or 0.001
  ang_tol = ang_tol or 0.1*DEG_TO_RAD

	local position_error = math.sqrt(
	( tr_check[1]-tr[1] )^2 +
	( tr_check[2]-tr[2] )^2 +
	( tr_check[3]-tr[3] )^2 )

	local angle_error = math.sqrt(
	util.mod_angle( tr_check[4]-tr[4] )^2 +
	util.mod_angle( tr_check[5]-tr[5] )^2 +
	util.mod_angle( tr_check[6]-tr[6] )^2 )

	-- If within tolerance, return true
  local in_tolerance = true
	if position_error>pos_tol then in_tolerance=false end
  if angle_error>ang_tol then in_tolerance=false end

--  if not in_tolerance then
if false then
    print("IK ERROR")
    print(string.format("tr0:%.2f %.2f %.2f %.2f %.2f %.2f tr:%.2f %.2f %.2f %.2f %.2f %.2f",
    tr_check[1],
    tr_check[2],
    tr_check[3],
    tr_check[4]*RAD_TO_DEG,
    tr_check[5]*RAD_TO_DEG,
    tr_check[6]*RAD_TO_DEG,
    tr[1],
    tr[2],
    tr[3],
    tr[4]*RAD_TO_DEG,
    tr[5]*RAD_TO_DEG,
    tr[6]*RAD_TO_DEG
    ))
    print(string.format("LArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
      vector.new(Body.get_larm_command_position())*RAD_TO_DEG     ) ))
    print(string.format("RArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
      vector.new(Body.get_rarm_command_position())*RAD_TO_DEG     ) ))
    print()
--    print(string.format("perr:%.4f aerr:%.2f",position_error, angle_error*Body.RAD_TO_DEG))
  end
	return in_tolerance
end

local function check_larm_bounds(qL)
  for i=1,nJointLArm do
    if qL[i]<servo.min_rad[indexLArm+i-1] or qL[i]>servo.max_rad[indexLArm+i-1] then
--      print("out of range",i,"at ",qL_target[i]*RAD_TO_DEG)
      return false
    end
  end
  return true
end

local function check_rarm_bounds(qR)
  for i=1,nJointRArm do
    if qR[i]<servo.min_rad[indexRArm+i-1] or qR[i]>servo.max_rad[indexRArm+i-1] then
--      print("out of range",i,"at ",qR_target[i]*RAD_TO_DEG)
      return false
    end
  end
  return true
end

--SJ: Now we consider waist angle and bodyTilt into FK/IK calculation
--Which is read from SHM




-- Take in joint angles and output an {x,y,z,r,p,yaw} table
-- SJ: Now separated into two functions to get rid of directly calling IK
Body.get_forward_larm = function(qL, bodyTilt, qWaist)
  local pLArm = Kinematics.l_arm_torso_7( qL,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3]
    )
  return pLArm
end

Body.get_forward_rarm = function(qR, bodyTilt, qWaist)
  local pRArm = Kinematics.r_arm_torso_7( qR,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3]
    )
  return pRArm
end

--Return the WRIST position (to test self collision)
Body.get_forward_lwrist = function(qL, bodyTilt, qWaist)
  local pLArm = Kinematics.l_wrist_torso( qL,
      bodyTilt or mcm.get_stance_bodyTilt(),
      qWaist or Body.get_waist_command_position())
  return pLArm
end

Body.get_forward_rwrist = function(qR, bodyTilt, qWaist)
  local pRArm = Kinematics.r_wrist_torso( qR,
      bodyTilt or mcm.get_stance_bodyTilt(),
      qWaist or Body.get_waist_command_position())
  return pRArm
end

Body.get_inverse_rwrist = function( qR, trR, rShoulderYaw, bodyTilt, qWaist)
  local qR_target = Kinematics.inverse_r_wrist(trR, qR,rShoulderYaw or qR[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return qR_target
end

Body.get_inverse_lwrist = function( qL, trL, lShoulderYaw, bodyTilt, qWaist)
  local qL_target = Kinematics.inverse_l_wrist(trL, qL, lShoulderYaw or qL[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return qL_target
end

Body.get_inverse_arm_given_wrist = function( q, tr, bodyTilt, qWaist)
  local q_target = Kinematics.inverse_arm_given_wrist(tr,q,
    bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return q_target
end


Body.get_inverse_larm = function( qL, trL, lShoulderYaw, bodyTilt, qWaist)
  local shoulder_flipped = 0
  if qL[2]>math.pi/2 then shoulder_flipped=1 end

  local qL_target = Kinematics.inverse_l_arm_7(
    trL,qL,
    lShoulderYaw or qL[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],
    mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3],
    shoulder_flipped
    )

  local trL_check = Kinematics.l_arm_torso_7(
    qL_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],
    mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3]
    )

  local passed = check_larm_bounds(qL_target) and check_ik_error( trL, trL_check)
  if passed then return qL_target end
end
--
Body.get_inverse_rarm = function( qR, trR, rShoulderYaw, bodyTilt, qWaist)
  local shoulder_flipped = 0
  if qR[2]<-math.pi/2 then shoulder_flipped=1 end
  local qR_target = Kinematics.inverse_r_arm_7(
    trR, qR,
    rShoulderYaw or qR[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],
    mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3],
    shoulder_flipped
    )

  local trR_check = Kinematics.r_arm_torso_7(
    qR_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],
    mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3]
    )

  local passed = check_rarm_bounds(qR_target) and check_ik_error( trR, trR_check)
  if passed then return qR_target end
end
--

-- If receiving data from a chain
local function chain_cb (c_skt)
  for _, msg in ipairs(c_skt:recv_all()) do
    print('DCM '..c_skt.obj.id..' | ', msg)
  end
end
local function imu_cb ()
  print('imu cb')
end
local function body_cb (b_skt)
	-- Externally call some sort of sync
  local msgs = b_skt:recv_all()
	for _, msg in ipairs(msgs) do
		for i, ch in ipairs(dcm_chs) do
      ch:send(msg)
    end
	end

end

function Body.entry ()
	-- Reset the tables
	dev_chs, dcm_chs, body_chs, body_poll = {}, {}, {}
	-- Start all the threads
	-- TODO: Check if already running as a separate process
	if Config.chain.enabled then
		for i, v in ipairs(Config.chain) do
			local ch, thread =
			si.new_thread(ROBOT_HOME..'/run_dcm.lua', 'dcm'..i, v)
			ch.callback = chain_cb
			ch.id = i
			table.insert(dev_chs, ch)
			table.insert(dcm_chs, ch)
			table.insert(body_chs, ch)
			thread:start()
		end
	end
	-- IMU
	if Config.imu.enabled then
		local imu_ch, imu_thread =
			si.new_thread(ROBOT_HOME..'/run_imu.lua', 'imu', v)
		imu_ch.callback = imu_cb
		table.insert(dev_chs, imu_ch)
		table.insert(body_chs, imu_ch)
		imu_thread:start()
	end
	-- Body requests
	-- Listens from everybody
	local body_ch = si.new_subscriber'body!'
	body_ch.callback = body_cb
	table.insert(body_chs, body_ch)
	-- Polling object
	body_poll = si.wait_on_channels(body_chs)
end

function Body.update ()
	-- Poll for events
	-- Return immediately if nothing happening
	-- NOTE: Most of the time, nothing will happen...
	body_poll:poll(1)
end

function Body.exit ()
	-- Tell the devices to exit cleanly
  for _,ch in pairs(dev_chs) do
		ch:send'exit'
		unix.usleep(1e4)
	end
end

----------------------
-- Webots compatibility


if IS_TESTING then
  print("TESTING")
  local Config     = require'Config'
  local jpeg       = require'jpeg'
  local png        = require'png'
  local udp        = require'udp'
  local mp         = require'msgpack'
  Body.entry = function ()
  end
  Body.update = function()



    local t = Body.get_time()

    local rad = dcm.get_actuator_command_position()
    dcm.set_sensor_position(rad)

  end
  Body.exit=function()
  end
  get_time = function()
    return global_clock
  end

  servo.min_rad = vector.new({
    -90,-80, -- Head
    -90, 0, -90, -160,      -180,-87,-180, --LArm
    -175,-175,-175,-175,-175,-175, --LLeg
    -175,-175,-175,-175,-175,-175, --RLeg
    -90,-87,-90,-160,       -180,-87,-180, --RArm
    -90,-45, -- Waist
    80,80,80,
    80,80,80,

    -60, -- Lidar pan
  })*DEG_TO_RAD

  servo.max_rad = vector.new({
    90, 80, -- Head
    160,87,90,0,     180,87,180, --LArm
    175,175,175,175,175,175, --LLeg
    175,175,175,175,175,175, --RLeg
    160,-0,90,0,     180,87,180, --RArm
    90,79, -- Waist
    45,45,45,
    45,45,45,
    60, -- Lidar pan
  })*DEG_TO_RAD

elseif IS_WEBOTS then

  local jointNames = {
    "Neck","Head", -- Head (Yaw,pitch)
    -- Left Arm
    "ShoulderL", "ArmUpperL", "LeftShoulderYaw",
    "ArmLowerL","LeftWristYaw","LeftWristRoll","LeftWristYaw2",
    -- Left leg
    "PelvYL","PelvL","LegUpperL","LegLowerL","AnkleL","FootL",
    -- Right leg
    "PelvYR","PelvR","LegUpperR","LegLowerR","AnkleR","FootR",
    --Right arm
    "ShoulderR", "ArmUpperR", "RightShoulderYaw","ArmLowerR",
    "RightWristYaw","RightWristRoll","RightWristYaw2",
    -- Waist
    "TorsoYaw","TorsoPitch",
    -- Gripper
    "l_grip", "l_trigger",
    "r_grip", "r_trigger",
    -- lidar movement
    "ChestLidarPan",
  }
  assert(nJoint==#jointNames,'bad jointNames!')

  servo.direction = vector.new({
    -1,-1, -- Head
    1,1,1,  1,  -1,-1,-1, --LArm
    --[[Yaw/Roll:]] -1, 1, --[[3 Pitch:]] 1,1,1, 1, --LLeg
    --[[Yaw/Roll:]] -1, 1, --[[3 Pitch:]] 1,1,1, 1, --RLeg
    1,1,1,  1,  -1,-1,-1, --RArm
    -- TODO: Check the gripper
    -1,1, -- Waist
    1,-1, -- left gripper
    -1,-1, -- right gripper

    1, -- Lidar pan
  })
  servo.rad_offset = vector.new({
    0,0, -- head
    -90,0,0,  0,  0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    -90,0,0,  0,  0,0,0,
    0,0,
    0,0,
    0,0,
    60,
  })*DEG_TO_RAD

  -- Default configuration (toggle during run time)
  local ENABLE_CAMERA = false
  local ENABLE_CHEST_LIDAR  = false
  local ENABLE_HEAD_LIDAR = false
  local ENABLE_KINECT = false
  local ENABLE_POSE   = false
  local ENABLE_IMU   = false

  local torch = require'torch'
  torch.Tensor = torch.DoubleTensor
  local carray = require'carray'
  local jpeg = require'jpeg'

	-- Publish sensor data
	local simple_ipc = require'simple_ipc'
	local mp = require'msgpack'
	local lidar0_ch = simple_ipc.new_publisher'lidar0'
	local lidar1_ch = simple_ipc.new_publisher'lidar1'

  local webots = require'webots'
  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(33,timeStep)
  local lidar_timeStep = math.max(25,timeStep)
  Body.timeStep = timeStep
  get_time = webots.wb_robot_get_time

  -- Setup the webots tags
  local tags = {}
  local t_last_error = -math.huge

  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable( 100 )
  local key_action = {
		h = function(override)
			if override~=nil then en=override else en=ENABLE_HEAD_LIDAR==false end
      if en==false then
        print(util.color('HEAD_LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.head_lidar)
        ENABLE_HEAD_LIDAR = false
      else
        print(util.color('HEAD_LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.head_lidar,lidar_timeStep)
        ENABLE_HEAD_LIDAR = true
      end
    end,
    l = function(override)
			if override~=nil then en=override else en=ENABLE_CHEST_LIDAR==false end
      if en==false then
        print(util.color('CHEST_LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.chest_lidar)
        ENABLE_CHEST_LIDAR = false
      else
        print(util.color('CHEST_LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.chest_lidar,lidar_timeStep)
        ENABLE_CHEST_LIDAR = true
      end
    end,
    c = function(override)
      if override~=nil then en=override else en=ENABLE_CAMERA==false end
      if en==false then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.head_camera)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.head_camera,camera_timeStep)
        ENABLE_CAMERA = true
      end
    end,
		p = function(override)
			if override~=nil then en=override else en=ENABLE_POSE==false end
      if en==false then
        print(util.color('POSE disabled!','yellow'))
        webots.wb_gps_disable(tags.gps)
	  		webots.wb_compass_disable(tags.compass)
				webots.wb_inertial_unit_disable(tags.inertialunit)
        ENABLE_POSE = false
      else
        print(util.color('POSE enabled!','green'))
        webots.wb_gps_enable(tags.gps, timeStep)
	  		webots.wb_compass_enable(tags.compass, timeStep)
				webots.wb_inertial_unit_enable(tags.inertialunit, timeStep)
        ENABLE_POSE = true
      end
    end,
		i = function(override)
			if override~=nil then en=override else en=ENABLE_IMU==false end
      if en==false then
        print(util.color('IMU disabled!','yellow'))
        webots.wb_accelerometer_disable(tags.accelerometer)
  			webots.wb_gyro_disable(tags.gyro)
        ENABLE_IMU = false
      else
        print(util.color('IMU enabled!','green'))
        webots.wb_accelerometer_enable(tags.accelerometer, timeStep)
  			webots.wb_gyro_enable(tags.gyro, timeStep)
        ENABLE_IMU = true
      end
    end,
		f = function(override)
			if override~=nil then en=override else en=ENABLE_FSR==false end
      if en==false then
        print(util.color('FSR disabled!','yellow'))
        webots.wb_touch_sensor_disable(tags.l_fsr)
  			webots.wb_touch_sensor_disable(tags.r_fsr)
        ENABLE_FSR = false
      else
        print(util.color('FSR enabled!','green'))
        webots.wb_touch_sensor_enable(tags.l_fsr, timeStep)
  			webots.wb_touch_sensor_enable(tags.r_fsr, timeStep)
        ENABLE_FSR = true
      end
    end,
  }

	Body.entry = function()

    -- Request @ t=0 to always be earlier than position reads
    dcm.set_trequest_position( vector.zeros(nJoint) )

		-- Grab the tags from the joint names
		tags.joints = {}
		for i,v in ipairs(jointNames) do
      local tag = webots.wb_robot_get_device(v)
			if tag>0 then
				webots.wb_motor_enable_position(tag, timeStep)
        webots.wb_motor_set_velocity(tag, 0.5);
        tags.joints[i] = tag
			end
		end

		-- Add Sensor Tags
		tags.accelerometer = webots.wb_robot_get_device("Accelerometer")
		tags.gyro = webots.wb_robot_get_device("Gyro")
		tags.gps = webots.wb_robot_get_device("GPS")
		tags.compass = webots.wb_robot_get_device("Compass")
		tags.inertialunit = webots.wb_robot_get_device("InertialUnit")
		tags.head_camera = webots.wb_robot_get_device("HeadCamera")
    tags.chest_lidar = webots.wb_robot_get_device("ChestLidar")
    tags.head_lidar = webots.wb_robot_get_device("HeadLidar")
		tags.l_fsr = webots.wb_robot_get_device("L_FSR")
    tags.r_fsr = webots.wb_robot_get_device("R_FSR")

		-- Enable or disable the sensors
		key_action.i(ENABLE_IMU)
		key_action.p(ENABLE_POSE)
		key_action.c(ENABLE_CAMERA)
		key_action.h(ENABLE_HEAD_LIDAR)
		key_action.l(ENABLE_CHEST_LIDAR)
		--key_action.k(ENABLE_KINECT)
		--key_action.f(ENABLE_FSR)

		-- Take a step to get some values
		webots.wb_robot_step(timeStep)
    webots.wb_robot_step(timeStep)

    for idx, jtag in ipairs(tags.joints) do
      if jtag>0 then
        local val = webots.wb_motor_get_position( jtag )
        local rad = servo.direction[idx] * val - servo.rad_offset[idx]
        dcm.sensorPtr.position[idx] = rad
        dcm.actuatorPtr.command_position[idx] = rad
        dcm.treadPtr.position[idx] = t
        dcm.twritePtr.command_position[idx] = t
      end
    end

	end
  Body.nop = function()
    -- Step only
    if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end
  end
	Body.update = function()

    local t = Body.get_time()

		local tDelta = .001 * Body.timeStep

    --Body.update_finger(tDelta)

		-- Set actuator commands from shared memory
		for idx, jtag in ipairs(tags.joints) do
			local cmd = Body.get_actuator_command_position(idx)
			local pos = Body.get_sensor_position(idx)
			-- TODO: What is velocity?
			local vel = 0 or Body.get_actuator_command_velocity(idx)
			local en  = 1 or Body.get_actuator_torque_enable(idx)
			local deltaMax = tDelta * vel
			-- Only update the joint if the motor is torqued on

			-- If the joint is moving
			-- Clamp the difference between commanded and actuated
			-- so that we don't have huge jumped
			-- NOTE: This *should* be handled by the simulator?
			local new_pos = cmd
			if vel > 0 then
				local delta = cmd - pos
				if delta > deltaMax then
					delta = deltaMax
				elseif delta < -deltaMax then
					delta = -deltaMax
				end
				new_pos = pos + delta
			end

			-- Only set in webots if Torque Enabled
			if en>0 and jtag>0 then
        local pos = servo.direction[idx] * (new_pos + servo.rad_offset[idx])
        --SJ: Webots is STUPID so we should set direction correctly to prevent flip
        local val = webots.wb_motor_get_position( jtag )

        if pos>val+math.pi then
          webots.wb_motor_set_position(jtag, pos-2*math.pi )
        elseif pos<val-math.pi then
          webots.wb_motor_set_position(jtag, pos+2*math.pi )
        else
          webots.wb_motor_set_position(jtag, pos )
        end
        dcm.twritePtr.command_position[idx] = t
      end
		end --for

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end

    if ENABLE_IMU then
      -- Accelerometer data (verified)
      local accel = webots.wb_accelerometer_get_values(tags.accelerometer)
      dcm.sensorPtr.accelerometer[1] = (accel[1]-512)/128
      dcm.sensorPtr.accelerometer[2] = (accel[2]-512)/128
      dcm.sensorPtr.accelerometer[3] = (accel[3]-512)/128

      -- Gyro data (verified)
      local gyro = webots.wb_gyro_get_values(tags.gyro)

      dcm.sensorPtr.gyro[1] = -(gyro[1]-512)/512*39.24
      dcm.sensorPtr.gyro[2] = -(gyro[2]-512)/512*39.24
      dcm.sensorPtr.gyro[3] = (gyro[3]-512)/512*39.24
    end

    -- FSR
    if ENABLE_FSR then
      dcm.sensorPtr.lfoot[1] = webots.wb_touch_sensor_get_value(tags.l_fsr)*4
      dcm.sensorPtr.rfoot[1] = webots.wb_touch_sensor_get_value(tags.r_fsr)*4
    end

    -- GPS and compass data
    -- Webots x is our y, Webots y is our z, Webots z is our x,
    -- Our x is Webots z, Our y is Webots x, Our z is Webots y
    if ENABLE_POSE then
      local gps     = webots.wb_gps_get_values(tags.gps)
      local compass = webots.wb_compass_get_values(tags.compass)
      local angle   = math.atan2( compass[3], compass[1] )
      local pose    = vector.pose{gps[3], gps[1], angle}
      --wcm.set_robot_pose( pose )
      wcm.set_robot_pose_gps( pose )

      local rpy = webots.wb_inertial_unit_get_roll_pitch_yaw(tags.inertialunit)

      --SJ: we need to remap rpy for webots
      dcm.sensorPtr.rpy[1],dcm.sensorPtr.rpy[2],dcm.sensorPtr.rpy[3] =
        rpy[2],rpy[1],-rpy[3]

      --[[
      print('rpy',unpack(rpy) )
      print('gps',unpack(gps) )
      print('compass',unpack(compass) )
      print('pose', pose )
      print()
      --]]
    end

		-- Update the sensor readings of the joint positions
		-- TODO: If a joint is not found?
		for idx, jtag in ipairs(tags.joints) do
			if jtag>0 then
				local val = webots.wb_motor_get_position( jtag )
				local rad = servo.direction[idx] * val - servo.rad_offset[idx]
        dcm.sensorPtr.position[idx] = rad
        dcm.treadPtr.position[idx] = t
			end
		end

    -- Grab a camera frame
    if ENABLE_CAMERA then
      local camera_fr = webots.to_rgb(tags.hand_camera)
      local w = webots.wb_camera_get_width(tags.hand_camera)
      local h = webots.wb_camera_get_height(tags.hand_camera)
      local jpeg_fr  = jpeg.compress_rgb(camera_fr,w,h)
    end
    -- Grab a lidar scan
    if ENABLE_CHEST_LIDAR then
      local w = webots.wb_camera_get_width(tags.chest_lidar)
      local lidar_fr = webots.wb_camera_get_range_image(tags.chest_lidar)
      local lidar_array = carray.float( lidar_fr, w )
			-- Send the message on the lidar channel
			local meta = {}
			meta.t     = Body.get_time()
			meta.n     = w
			meta.res   = 360 / 1440
			meta.pose  = wcm.get_robot_pose()
			lidar0_ch:send{mp.pack(meta),tostring(lidar_array)}
    end
    -- Grab a lidar scan
    if ENABLE_HEAD_LIDAR then
      local w = webots.wb_camera_get_width(tags.head_lidar)
      local lidar_fr = webots.wb_camera_get_range_image(tags.head_lidar)
      local lidar_array = carray.float( lidar_fr, w )
			-- Send the message on the lidar channel
			local meta = {}
			meta.t     = Body.get_time()
			meta.n     = w
			meta.res   = 360 / 1440
			meta.pose  = wcm.get_robot_pose()
			lidar1_ch:send{mp.pack(meta),tostring(lidar_array)}
    end

    -- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
		local key_toggle = key_action[key_char_lower]
    if key_toggle and t-t_last_keypress>1 then
      key_toggle()
      t_last_keypress = t
    end

	end -- update

	Body.exit = function()
	end
	Body.tags = tags

  --Touchdown check for webots
  --Use FSR sensors

  local FSR_threshold = 200
  Body.get_lfoot_touched = function()
    local LFSR = dcm.get-sensor_lfoot()
    if (LFSR[1]+LFSR[2]+LFSR[3]+LFSR[4])>FSR_threshold then
      return true
    else
      return false
    end
  end
  Body.get_rfoot_touched = function()
    local RFSR = dcm.get-sensor_rfoot()
    if (RFSR[1]+RFSR[2]+RFSR[3]+RFSR[4])>FSR_threshold then
      return true
    else
      return false
    end
  end
else -- webots check
  --Touchdown check for actual robot
  --Force torque sensor based
  Body.get_lfoot_touched = function()
    return false
  end
  Body.get_rfoot_touched = function()
    return false
  end
end


-- Exports for use in other functions
Body.get_time = get_time
-- Real THOR-OP (Cenatur uses ankles for wheels, maybe?)
Body.indexHead = indexHead   -- Head: 1 2
Body.nJointHead = nJointHead
Body.indexLArm = indexLArm   --LArm: 5 6 7 8 9 10
Body.nJointLArm = nJointLArm
Body.indexLLeg = indexLLeg  --LLeg: 11 12 13 14 15 16
Body.nJointLLeg = nJointLLeg
Body.indexRLeg = indexRLeg  --RLeg: 17 18 19 20 21 22
Body.nJointRLeg = nJointRLeg
Body.indexRArm = indexRArm  --RArm: 23 24 25 26 27 28
Body.nJointRArm = nJointRArm
Body.indexWaist = indexWaist  --Waist: 3 4
Body.nJointWaist = nJointWaist
-- 6 Fingers for gripping
-- 1 servo for lidar panning
Body.indexLGrip = indexLGrip
Body.nLGrip = nLGrip
Body.indexRGrip = indexRGrip
Body.nRGrip = nRGrip
Body.indexLidar = indexLidar
Body.nJointLidar = nJointLidar
Body.nJoint = nJoint
Body.jointNames = jointNames
Body.parts = parts
Body.inv_parts = inv_parts

Body.Kinematics = Kinematics

-- For supporting the THOR repo
require'mcm'
Body.set_walk_velocity = mcm.set_walk_vel

---------------------------------------------
-- New hand API
-- Positive force value for closing
-- Negative force value for openning
---------------------------------------------


--Used only for webots

if IS_WEBOTS then
  Body.finger_target={0,0,0,0}
  Body.finger_pos={0,0,0,0}

  Body.control_finger= function(finger_index,force)
    if force>0 then
      Body.finger_target[finger_index] = 1 --close
    elseif force<0 then
      Body.finger_target[finger_index] = 0 --open
    end
  end
  Body.update_finger = function(dt)
    Body.finger_pos = util.approachTol(
      Body.finger_pos,Body.finger_target,
      {2,2,2,2},dt)

    Body.set_lgrip_percent(Body.finger_pos[2])

    Body.set_rgrip_percent(Body.finger_pos[4])
  end

  Body.move_lgrip1 = function(force) Body.control_finger(1, force) end
  Body.move_lgrip2 = function(force) Body.control_finger(2, force) end
  Body.move_rgrip1 = function(force) Body.control_finger(3, force) end
  Body.move_rgrip2 = function(force) Body.control_finger(4, force) end

else
  Body.move_lgrip1 = Body.set_ltrigger_command_torque
  Body.move_lgrip2 = Body.set_lgrip_command_torque
  Body.move_rgrip1 = Body.set_rtrigger_command_torque
  Body.move_rgrip2 = Body.set_rgrip_command_torque
end















return Body
