--------------------------------
-- Body abstraction for THOR-OP
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
--------------------------------
--assert(ffi, 'Need LuaJIT to run. Lua support in the future')

-- Utilities
local vector = require'vector'
local util = require'util'
local si = require'simple_ipc'
local Kinematics = require'THOROPKinematics'
local mpack  = require'msgpack.MessagePack'.pack
require'dcm'

local Body = {}
local dcm_ch = si.new_publisher'dcm!'
local get_time = require'unix'.time
local usleep = require'unix'.usleep
local vslice = require'vector'.slice

-- Body sensors
local nx_registers = require'libDynamixel'.nx_registers
for sensor, n_el in pairs(dcm.sensorKeys) do
	--local cur = dcm['get_sensor_'..sensor]()
	--local n_el = type(cur)=='table' and #cur or 1
  local is_motor = nx_registers[sensor]
  local ptr, ptr_t
  if dcm.sensorPtr then
    ptr = dcm.sensorPtr[sensor]
    ptr_t = dcm.tsensorPtr[sensor]
  end
  local function get(idx1, idx2, needs_wait)
    --for some reason, webot makes this code wait, slowing down simulation a lot
    if IS_WEBOTS then needs_wait=false end

		local start, stop = idx1 or 1, idx2 or n_el
    if is_motor and needs_wait then
			local ids = {}
			for id = start, stop do ids[id] = true end
			dcm_ch:send(mpack({rd_reg=sensor, ids=ids}))
			-- 100Hz assumed for the wait period to be in SHM
			unix.usleep(1e4)
    end
  	-- For cdata, use -1
		-- Return the time of the reading
	  return vslice(ptr, start-1, stop-1), vslice(ptr_t, start-1, stop-1)
	end

  Body['get_'..sensor] = get
  -- Anthropomorphic access to dcm
	-- TODO: get_lleg_rpy is illegal, for instance
  for part, jlist in pairs(Config.parts) do
	  local not_synced = sensor~='position'
    local idx1, idx2 = jlist[1], jlist[#jlist]
    Body['get_'..part:lower()..'_'..sensor] = function(idx)
			return get(idx1, idx2, not_synced)
    end -- Get
  end
	-- End anthropomorphic
end

-- Body actuators
for actuator, n_el in pairs(dcm.actuatorKeys) do
	-- Only command_position is constantly synced
	-- Other commands need to be specially sent to the Body
  -- TODO: Check the torque usage in NX motors...
	local not_synced = not (actuator=='command_position' or actuator=='command_torque')
  local ptr = dcm.actuatorPtr and dcm.actuatorPtr[actuator]
  local idx
	local function set(val, idx1, idx2)
		local changed_ids = {}
		-- cdata is -1
		if idx2 then
			if type(val)=='number' then
				for idx=idx1, idx2 do
					changed_ids[idx] = true
					ptr[idx - 1] = val
				end
			else
				for i,v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					if idx>idx2 then break else ptr[idx - 1] = v end
				end
			end
		elseif idx1 then
			if type(val)=='number' then
				changed_ids[idx1] = true
				ptr[idx1 - 1] = val
			else
				for i, v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					ptr[idx - 1] = v
				end
			end
		else
			-- No index means set all actuators... Uncommon
			if type(val)=='number' then
				for i=0, n_el-1 do
					changed_ids[i + 1] = true
					ptr[i] = val
				end
			else
				for i, v in ipairs(val) do
					changed_ids[i] = true
					ptr[i - 1] = v
				end
			end
		end
		-- Send msg to the dcm, just string of the id
		if not_synced then
			dcm_ch:send(mpack({wr_reg=actuator, ids=changed_ids}))
		end
	end
	local function get(idx1, idx2, needs_wait)
		idx1 = idx1 or 1
		idx2 = idx2 or n_el
    if needs_wait then
			local ids = {}
			for id = idx1, idx2 do ids[id] = true end
			dcm_ch:send(mpack({rd_reg=actuator, ids=ids}))
			-- 100Hz assumed for the wait period to be in SHM
			unix.usleep(1e4)
    end
		-- For cdata, use -1
		return vslice(ptr, idx1 - 1, idx2 - 1)
	end
	-- Export
  Body['set_'..actuator] = set
  Body['get_'..actuator] = get
  --------------------------------
  -- Anthropomorphic access to dcm
  -- TODO: Do not use string concatenation to call the get/set methods of Body
  for part, jlist in pairs(Config.parts) do
		local idx1, idx2, idx = jlist[1], jlist[#jlist], nil
		Body['get_'..part:lower()..'_'..actuator] = function(idx)
			local needs_wait = not (actuator=='command_position')
			if idx then return get(jlist[idx], needs_wait) else return get(idx1, idx2, needs_wait) end
		end
		Body['set_'..part:lower()..'_'..actuator] = function(val, i)
			if i then
        return set(val, jlist[i])
      else
        -- Check the ankle on a full set of lleg/rleg
        -- Do not set the last 2 (ankle) values
        --[[
        if part=='LLeg' then
          if (val[6]>0 and val[5]>0) or (val[6]>0 and val[5]>0)
          then
            return set(val, idx1, idx2-2)
          end
        elseif part=='RLeg' then
          if (val[6]>0 and val[5]>0) or (val[6]>0 and val[5]>0)
          then
            return set(val, idx1, idx2-2)
          end
        end
        --]]
        return set(val, idx1, idx2)
      end
		end
  end
	-- End anthropomorphic
end

function Body.entry() end
function Body.update() end
function Body.exit() end

---
-- Special functions
---
function Body.enable_read(chain)
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=true}))
end
function Body.disable_read(chain)
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=false}))
end

----------------------
-- Add the gripper API
----------------------
local lgrip1_id, lgrip2_id, lgrip3_id = unpack(Config.parts.LGrip)
local rgrip1_id, rgrip2_id, rgrip3_id = unpack(Config.parts.RGrip)
local lgrip_ids = {[lgrip1_id] = true, [lgrip2_id] = true, [lgrip3_id]=true}
local rgrip_ids = {[rgrip1_id] = true, [rgrip2_id] = true, [rgrip3_id]=true}
function Body.set_lgrip_mode(mode)
	local msg = {wr_reg='torque_mode', ids=lgrip_ids}
	if mode=='torque' then
		msg.val = {[lgrip1_id] = 1, [lgrip2_id] = 1, [lgrip3_id] = 1}
	elseif mode=='position' then
		msg.val = {[lgrip1_id] = 0, [lgrip2_id] = 0, [lgrip3_id] = 0}
	end
	dcm_ch:send(mpack(msg))
end
function Body.set_rgrip_mode(mode)
	local msg = {wr_reg='torque_mode', ids=rgrip_ids}
	if mode=='torque' then
		msg.val = {[rgrip1_id] = 1, [rgrip2_id] = 1, [rgrip3_id] = 1}
	elseif mode=='position' then
		msg.val = {[rgrip1_id] = 0, [rgrip2_id] = 0, [rgrip3_id] = 0}
	end
	dcm_ch:send(mpack(msg))
end
----------------------

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


--TODO:fix here
--[[
-- will we ever use lower-dof arm for anything?
local nJointLArm = 7
local nJointRArm = 7
--where's servo.min_rad defined now?
-- It is in Config.servo, Config_THOROP_Robot.lua
--]]

local function check_larm_bounds(qL)
  --SJ: now we don't hacve nJointLArm definition
--[[
  print("check larm bound, nJointLArm:",nJointLArm)
  for i=1,nJointLArm do
    if qL[i]<servo.min_rad[indexLArm+i-1] or qL[i]>servo.max_rad[indexLArm+i-1] then
--      print("out of range",i,"at ",qL_target[i]*RAD_TO_DEG)
      return false
    end
  end
  --]]
  return true
end

local function check_rarm_bounds(qR)
  --[[
  for i=1,nJointRArm do
    if qR[i]<servo.min_rad[indexRArm+i-1] or qR[i]>servo.max_rad[indexRArm+i-1] then
--      print("out of range",i,"at ",qR_target[i]*RAD_TO_DEG)
      return false
    end
  end
  --]]
  return true
end

--]]

--SJ: Now we consider waist angle and bodyTilt into FK/IK calculation
--Which is read from SHM


-- Take in joint angles and output an {x,y,z,r,p,yaw} table
-- SJ: Now separated into two functions to get rid of directly calling IK
Body.get_forward_larm = function(qL, bodyTilt, qWaist,ignore_hand_offset )
  if ignore_hand_offset then
    return Kinematics.l_arm_torso_7( qL,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    0,0,0)
  end
  local pLArm = Kinematics.l_arm_torso_7( qL,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3]
    )
  return pLArm
end

Body.get_forward_rarm = function(qR, bodyTilt, qWaist,ignore_hand_offset )
  if ignore_hand_offset then
    return Kinematics.r_arm_torso_7( qR,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    0,0,0)
  end
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


Body.get_inverse_larm = function( qL, trL, lShoulderYaw, bodyTilt, qWaist,ignore_hand_offset)
  local shoulder_flipped = 0
  if qL[2]>math.pi/2 then shoulder_flipped=1 end
  local hand_offset = mcm.get_arm_lhandoffset()
  if ignore_hand_offset then hand_offset={0,0,0} end

  local qL_target = Kinematics.inverse_l_arm_7(
    trL,qL,
    lShoulderYaw or qL[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    hand_offset[1],hand_offset[2],hand_offset[3],
    shoulder_flipped
    )

  local trL_check = Kinematics.l_arm_torso_7(
    qL_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    hand_offset[1],hand_offset[2],hand_offset[3]
    )

  local passed = check_larm_bounds(qL_target) and check_ik_error( trL, trL_check)
  if passed then return qL_target end
end
--
Body.get_inverse_rarm = function( qR, trR, rShoulderYaw, bodyTilt, qWaist,ignore_hand_offset)
  local shoulder_flipped = 0
  if qR[2]<-math.pi/2 then shoulder_flipped=1 end
  local hand_offset = mcm.get_arm_lhandoffset()
  if ignore_hand_offset then hand_offset={0,0,0} end

  local qR_target = Kinematics.inverse_r_arm_7(
    trR, qR,
    rShoulderYaw or qR[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    hand_offset[1],hand_offset[2],hand_offset[3],
    shoulder_flipped
    )

  local trR_check = Kinematics.r_arm_torso_7(
    qR_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    hand_offset[1],hand_offset[2],hand_offset[3])

  local passed = check_rarm_bounds(qR_target) and check_ik_error( trR, trR_check)
  if passed then return qR_target end
end
--

---------------------------------------------
-- New hand API
-- Positive force value for closing
-- Negative force value for openning
---------------------------------------------
Body.move_lgrip1 = Body.set_ltrigger_command_torque
  Body.move_lgrip2 = Body.set_lgrip_command_torque
  Body.move_rgrip1 = Body.set_rtrigger_command_torque
  Body.move_rgrip2 = Body.set_rgrip_command_torque

----------------------
-- Webots compatibility
if IS_WEBOTS then
	local WebotsBody
--  local webots = require'webots'
--now a global variable
  webots = require'webots'
	local ImageProc = require'ImageProc'

  Body.enable_read = function(chain) end
  Body.disable_read = function(chain) end
  Body.exit = function() end

  webots.wb_robot_init()
  Body.timeStep = webots.wb_robot_get_basic_time_step()  
  WebotsBody = require'WebotsBody'
  
	-- Check if we are using the OLD api
  last_webots_time=webots.wb_robot_get_time()
	
	function Body.entry()
		WebotsBody.entry(Body)    
	end

  function Body.update()
    WebotsBody.update(Body)
  end
  
  get_time = webots.wb_robot_get_time
  --Force torque sensor based
  Body.get_lfoot_touched = function() return false end
  Body.get_rfoot_touched = function() return false end
	
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
end

-- Exports for use in other functions
Body.get_time = get_time
Body.nJoint = nJoint
Body.jointNames = jointNames
Body.parts = Config.parts
Body.Kinematics = Kinematics



--SJ: those function are added as the joint-level waist yaw position can be 360 degree off
Body.get_safe_waist_position = function()
  local qWaist = Body.get_waist_position()
  qWaist[1] = util.mod_angle(qWaist[1])
  return qWaist
end

Body.get_safe_waist_command_position = function()
  local qWaist = Body.get_waist_command_position()
  qWaist[1] = util.mod_angle(qWaist[1])
  return qWaist
end

Body.set_safe_waist_command_position = function(qWaist)
  local qWaistSafe={qWaist[1],qWaist[2]}
  qWaistSafe[1] = math.max(math.min(qWaistSafe[1],90*DEG_TO_RAD), -90*DEG_TO_RAD)
  qWaistSafe[2] = 0 --fix pitch angle here  
  local qWaistCommand = Body.get_waist_command_position()
  local qWaistDiff = util.mod_angle(qWaistSafe[1]-qWaistCommand[1])
  qWaistSafe[1] = qWaistCommand[1]+qWaistDiff
  Body.set_waist_command_position(qWaistSafe)
end


--These function should be used instead of set_xarm_command_position, etc

Body.get_safe_larm_position= function()
  local qLArmRaw = Body.get_larm_position()
  local armBias = mcm.get_arm_bias()
  return qLArmRaw - vector.slice(armBias,1,7)
end

Body.get_safe_rarm_position= function()
  local qRArmRaw = Body.get_rarm_position()
  local armBias = vector.new(mcm.get_arm_bias())
  return qRArmRaw - vector.slice(armBias,8,14)
end

Body.get_safe_larm_command_position= function()
  local qLArmRaw = Body.get_larm_command_position()
  local armBias = mcm.get_arm_bias()
  return qLArmRaw - vector.slice(armBias,1,7)
end

Body.get_safe_rarm_command_position= function()
  local qLArmRaw = Body.get_larm_command_position()
  local armBias = mcm.get_arm_bias()
  return qRArmRaw - vector.slice(armBias,8,14)
end

Body.set_safe_larm_command_position= function(qLArm)
  local armBias = mcm.get_arm_bias()
  Body.set_larm_command_position(vector.new(qLArm)+vector.slice(armBias,1,7) )
end

Body.set_safe_rarm_command_position= function(qRArm)
  local armBias = mcm.get_arm_bias()
  Body.set_rarm_command_position(vector.new(qRArm)+vector.slice(armBias,8,14) )
end







--SJ: I have moved this function to body as it is commonly used in many locations
--Reads current leg and torso position from SHM
require'mcm'
Body.get_torso_compensation= function (qLArm, qRArm, qWaist)
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local zLeg = mcm.get_status_zLeg()
  local zSag = mcm.get_walk_zSag()
  local zLegComp = mcm.get_status_zLegComp()
  local zLeft,zRight = zLeg[1]+zSag[1]+zLegComp[1],zLeg[2]+zSag[2]+zLegComp[2]
  local aShiftX = mcm.get_walk_aShiftX()
  local aShiftY = mcm.get_walk_aShiftY()


  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)
  local global_angle = mcm.get_walk_global_angle()

  zRight = zRight + math.tan(global_angle[1])*uRightTorso[2]
  zLeft = zLeft - math.tan(global_angle[1])*uRightTorso[2]
  aShiftX[1],aShiftX[2] =aShiftX[1]+global_angle[1],aShiftX[2]+global_angle[1]

--pitch adaptation test
----[[
  zRight = zRight + math.tan(global_angle[2])*uRightTorso[1]
  zLeft = zLeft - math.tan(global_angle[2])*uRightTorso[1]
  aShiftY[1],aShiftY[2] =aShiftY[1]+global_angle[2],aShiftY[2]+global_angle[2]
--]]

  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})  
  local count,revise_max = 1,4
  local adapt_factor = 1.0


  local footLift = mcm.get_walk_footlift()
  local footlifttypeL,footlifttypeR, footliftL, footliftR = 0,0
  if mcm.get_walk_heeltoewalk()==1 then
    if footLift[1]>0 then 
      footlifttypeL = -1 --heellift
    else
      footlifttypeL = 1 --toelift
    end
    if footLift[2]>0 then 
      footlifttypeR = -1 --heellift
    else
      footlifttypeR = 1 --toelift
    end
    footliftL = math.abs(footLift[1])
    footliftR = math.abs(footLift[2])
  end

  local leftSupportRatio = mcm.get_status_leftSupportRatio()

 --Initial guess 
  local uTorsoAdapt = util.pose_global(vector.new({-Config.walk.torsoX,0,0}),uTorso)
  local pTorso = vector.new({
    uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
    0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})

  local qLLegCurrent = Body.get_lleg_command_position()
  local qRLegCurrent = Body.get_rleg_command_position()

  local qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso,aShiftX,aShiftY , Config.birdwalk or 0,
    qLLegCurrent, qRLegCurrent, footlifttypeL, footlifttypeR, footliftL, footliftR)
    
  local massL,massR = 0,0 --for now

  -------------------Incremental COM filtering
  while count<=revise_max do
    local qLLeg = vector.slice(qLegs,1,6)
    local qRLeg = vector.slice(qLegs,7,12)

  --Now we compensate for leg masses too (for single support cases)
    com = Kinematics.calculate_com_pos(qWaist,qLArm,qRArm,qLLeg,qRLeg,
          massL, massR,0, Config.birdwalk or 0)

    local uCOM = util.pose_global(
      vector.new({com[1]/com[4], com[2]/com[4],0}),uTorsoAdapt)

   uTorsoAdapt[1] = uTorsoAdapt[1]+ adapt_factor * (uTorso[1]-uCOM[1])
   uTorsoAdapt[2] = uTorsoAdapt[2]+ adapt_factor * (uTorso[2]-uCOM[2])
   local pTorso = vector.new({
            uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})
      qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY, Config.birdwalk or 0,
          qLLegCurrent, qRLegCurrent, footlifttypeL, footlifttypeR, footliftL, footliftR)
   count = count+1
  end
  local uTorsoOffset = util.pose_relative(uTorsoAdapt, uTorso)
  return {uTorsoOffset[1],uTorsoOffset[2]}, qLegs, com[3]/com[4]
end

return Body
