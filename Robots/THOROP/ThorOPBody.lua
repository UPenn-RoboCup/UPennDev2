-- THOR OP Kinematics
-- Dynamixel definitions
local libDynamixel = require'libDynamixel2'

-- Shared memory for the joints
require'jcm'

-- Utilities
require'vector'

local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi

local Body = {}

--------------------------------
-- Shared memory layout
--[[
-- Real THOR-OP
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexSpine = 3  --Spine: 3 4
local nJointSpine = 2
local indexLArm = 5   --LArm: 5 6 7 8 9 10
local nJointLArm = 6 		
local indexLLeg = 11  --LLeg: 11 12 13 14 15 16
local nJointLLeg = 6
local indexRLeg = 17  --RLeg: 17 18 19 20 21 22
local nJointRLeg = 6
local indexRArm = 23  --RArm: 23 24 25 26 27 28
local nJointRArm = 6
-- Auxiliary servos
-- 6 Fingers for gripping
-- 1 servo for lidar panning
local indexAux = 29 
local nJointAux = 7
local nJoint = 35
--]]
----[[
-- THOR-OP Centaur
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexSpine = 3  --Spine: 3 4
local nJointSpine = 2
local indexLArm = 5   --LArm: 5 6 7 8 9 10
local nJointLArm = 6 		
local indexLLeg = 11  --LWheel: 11
local nJointLLeg = 1
local indexRLeg = 12  --RWheel: 12
local nJointRLeg = 1
local indexRArm = 13  --RArm: 13 14 15 16 17 18
local nJointRArm = 6
-- Auxiliary servos
-- 6 Fingers for gripping
-- 1 servo for lidar panning
local indexAux = 19
local nJointAux = 7
local nJoint = 25
local indexLGrip = 19
local nLGrip = 3
local indexRGrip = 22
local nRGrip = 3
--]]
assert(nJoint==(indexAux+nJointAux)-indexHead,'Bad joint counting!')
Body.nJoint = nJoint
local parts = {
  ['Head']=vector.count(indexHead,nJointHead),
  ['Spine']=vector.count(indexSpine,nJointSpine),
  ['LArm']=vector.count(indexLArm,nJointLArm),
  ['LLeg']=vector.count(indexLLeg,nJointLLeg),
  ['RLeg']=vector.count(indexRLeg,nJointRLeg),
  ['RArm']=vector.count(indexRArm,nJointRArm),
  ['Aux']=vector.count(indexAux,nJointAux)
}
local inv_parts = {}
for name,list in pairs(parts) do
  for i,idx in ipairs(list) do
    inv_parts[idx] = name
  end
end
Body.parts = parts
Body.inv_parts = inv_parts
--------------------------------
-- Servo parameters
local servo = {}

-- shm joint id to dynamixel motor id
servo.joint_to_motor={
  27,28,  --Head
  25,26,  -- Spine
  1,3,5,7,9,11, --LArm
  -- TODO: No legs!
  20, -- left wheel
  19, -- right wheel
  2,4,6,8,10,12,  --RArm
  14,16,18, -- left gripper
  13,15,17, -- right gripper
  36, -- Lidar pan
  
}
assert(#servo.joint_to_motor==nJoint,'Bad servo id map!')

local motor_parts = {}
for name,list in pairs(parts) do
  motor_parts[name] = vector.zeros(#list)
  for i,idx in ipairs(list) do
    motor_parts[name][i] = servo.joint_to_motor[idx]
  end
end
Body.motor_parts = motor_parts

-- Make the reverse map
servo.motor_to_joint={}
for j,m in ipairs(servo.joint_to_motor) do
  servo.motor_to_joint[m] = j
end

servo.direction = vector.new({
  1,1, -- Head
  1,1, -- Spine
  1,1,1,-1,1,-1, --LArm
  -- TODO: No legs yet!
  1, --LLeg
  1, --LArm
  -1,1,1,1, 1,-1, --RArm
  -- TODO: Check the gripper
  1,-1,1, -- left gripper
  1,1,1, -- right gripper
  1, -- Lidar pan
  
  })
assert(#servo.direction==nJoint,'Bad servo direction!')

--http://support.robotis.com/en/product/dynamixel_pro/control_table.htm#Actuator_Address_611
-- TODO: Use some loop based upon MX/NX
-- TODO: some pros are different
servo.steps = 2 * 251000 * vector.ones(nJoint)
-- Aux is MX
for _,idx in ipairs( parts['Aux'] ) do
  servo.steps[idx] = 4096
end

-- Convienence tables to go between steps and radians
servo.moveRange = 360 * DEG_TO_RAD * vector.ones(nJoint)
servo.step_bias = vector.zeros(nJoint)
servo.to_radians = vector.zeros(nJoint)
servo.to_steps = vector.zeros(nJoint)
servo.step_zero = vector.zeros(nJoint)
servo.max_step = vector.zeros(nJoint)
servo.min_step = vector.zeros(nJoint)
servo.max_rad = vector.zeros(nJoint)
servo.min_rad = vector.zeros(nJoint)
for i,nsteps in ipairs(servo.steps) do
  servo.step_zero[i] = nsteps/2
  servo.to_radians[i] = servo.moveRange[i] / nsteps
  servo.to_steps[i] = nsteps / servo.moveRange[i]
  servo.min_step[i] = 0
  servo.max_step[i] = servo.steps[i]
  servo.min_rad[i] = servo.to_radians[i] * servo.min_step[i]
  servo.max_rad[i] = servo.to_radians[i] * servo.max_step[i]
end

-- Some overrides
-- Gripper has different min/max limits
for _,idx in ipairs(Body.parts['Aux']) do
  servo.min_rad[idx] = -10*DEG_TO_RAD
  servo.max_rad[idx] = 30*DEG_TO_RAD
  servo.min_step[idx] = servo.min_step[idx] * servo.to_steps[idx]
  servo.max_step[idx] = servo.max_step[idx] * servo.to_steps[idx]
end

-- Radian to step, using offsets and biases
local make_joint_step = function( idx, radian, safe )
  local step = math.floor(servo.direction[idx] * radian * servo.to_steps[idx]
    + servo.step_bias[idx] + servo.step_zero[idx])
  if not safe then return step end
  return math.min(math.max(step, servo.min_step[idx]),servo.max_step[idx])
  --return math.min(math.max(step, 0), servo.steps[idx]-1);
end
Body.make_joint_step = make_joint_step
-- Step to radian
local make_joint_radian = function( idx, step )
  local radian = servo.direction[idx] * servo.to_radians[idx] * 
    (step - servo.step_zero[idx] - servo.step_bias[idx])
  return radian
end
Body.make_joint_radian = make_joint_radian

Body.servo = servo

--------------------------------
-- Motor wizard Standard convenience functions to access jcm
Body.get_joint_position = function(idx)
  return jcm.sensorPtr.position[idx]
end
Body.set_joint_position = function(idx,val)
  jcm.sensorPtr.position[idx] = val
end

Body.set_joint_command = function(idx,val)
  jcm.actuatorPtr.command[idx] = val
end
Body.get_joint_command = function(idx)
  return jcm.actuatorPtr.command[idx]
end

--------------------------------
-- Standard Convenience functions to access jcm
for part,jlist in pairs(Body.parts) do
  local a = jlist[1]
  local b = jlist[#jlist]
  Body['get_'..part:lower()..'_position'] = function(idx)
    if idx then return jcm.sensorPtr.position[ jlist[idx] ] end
    return jcm.sensorPtr.position:table( a, b )
  end -- Get
  Body['set_'..part:lower()..'_command'] = function(val,idx)
    if type(val)=='number' then
      if idx then jcm.actuatorPtr.command[ jlist[idx] ] = val; return; end
      for _,l in ipairs(jlist) do
        jcm.actuatorPtr.command[l] = val
      end
    else
      for i,l in ipairs(jlist) do
        jcm.actuatorPtr.command[l] = val[i]
      end
    end -- if number
  end -- Set
end

--------------------------------
-- Additional Convenience functions to access jcm
for part,jlist in pairs(Body.parts) do
  
  local a = jlist[1]
  local b = jlist[#jlist]
  
  Body['get_'..part:lower()..'_command'] = function(idx)
    if idx then return jcm.actuatorPtr.command[ jlist[idx] ] end
    return jcm.actuatorPtr.command:table( a, b )
  end -- Get
  Body['set_'..part:lower()..'_position'] = function(val,idx)
    if type(val)=='number' then
      if idx then jcm.sensorPtr.position[ list[idx] ] = val; return; end
      for _,jid in ipairs(jlist) do
        jcm.sensorPtr.position[jid] = val
      end
    else
      for i,jid in ipairs(jlist) do
        jcm.sensorPtr.position[jid] = val[i]
      end
    end -- if number
  end -- Set
  
  
  Body['get_'..part:lower()..'_torque_enable'] = function(idx)
    if idx then return jcm.actuatorPtr.torque_enable[ list[idx] ] end
    return jcm.actuatorPtr.torque_enable:table( a, b )
  end -- Get
  Body['set_'..part:lower()..'_torque_enable'] = function(val,idx)
    if type(val)=='number' then
      if idx then jcm.sensorPtr.position[ jlist[idx] ] = val; return; end
      for _,l in ipairs(jlist) do
        jcm.actuatorPtr.torque_enable[l] = val
      end
    else
      for i,l in ipairs(jlist) do
        jcm.actuatorPtr.torque_enable[l] = val[i]
      end
    end -- if number
  end -- Set
  
  
end

--------------------------------
-- Convenience functions
-- Use libDynamixel to have access to more settings, via instructions
for k,v in pairs(libDynamixel.nx_registers) do
  local get_func = libDynamixel['get_nx_'..k]
  local set_func = libDynamixel['get_nx_'..k]
  for part,mlist in pairs(Body.motor_parts) do
    local jlist = Body.parts[part]
    local a = jlist[1]
    local b = jlist[#jlist]
    Body['get_'..part..'_'..k..'_packet'] = function()
      return get_func(mlist)
    end
    Body['set_'..part..'_'..k..'_packet'] = function()
      return set_func(mlist,jcm.actuatorPtr.command:table( a, b ))
    end
  end
end

-- Gripper and lidar actuator are MX
-- TODO: make general somehow
for k,v in pairs(libDynamixel.mx_registers) do
  local get_func = libDynamixel['get_mx_'..k]
  local set_func = libDynamixel['set_mx_'..k]
  local mlist = Body.motor_parts['Aux']
  local jlist = Body.parts['Aux']
  local a = jlist[1]
  local b = jlist[#jlist]
  Body['get_aux_'..k..'_packet'] = function()
    return get_func(mlist)
  end
  Body['set_aux_'..k..'_packet'] = function()
    local vals = jcm.actuatorPtr[k]:table( a, b )
    return set_func( mlist, vals )
  end
  if k=='command' then
    --print'overwriting command'
    Body['set_aux_command_packet'] = function()
      -- Shm has radians
      local vals = jcm.actuatorPtr.command:table( a, b )
      -- Convert to 
      for i,idx in ipairs(jlist) do
        vals[i] = make_joint_step(idx,vals[i],true)
      end
      return set_func( mlist, vals )
    end
  end
end

-- TODO: should be in body or a Grip module?
-- Grip module may have more advanced techniques...
-- More gripper functions
Body.set_lgrip_percent = function( percent )
  percent = math.min(math.max(percent,0),1)
  -- Convex combo
  for idx=indexLGrip,indexLGrip+nLGrip-1 do
    local radian = percent*servo.min_rad[idx] + (1-percent)*servo.max_rad[idx]
    jcm.actuatorPtr.command[idx] = radian
  end
end
Body.set_rgrip_percent = function( percent )
  percent = math.min(math.max(percent,0),1)
  -- Convex combo
  for idx=indexRGrip,indexRGrip+nRGrip-1 do
    local radian = percent * servo.min_rad[idx] + (1-percent)*servo.max_rad[idx]
    jcm.actuatorPtr.command[idx] = radian
  end
end

----------------------
-- Inverse Kinematics
local Kinematics = require'THOROPKinematics'

-- take in a transform and output joint angles
Body.get_larm_ik = function( tr )
  
  local qLArm = get_larm_position();
  local trLArm = Kinematics.inverse_l_arm(tr, qLArm)
  
  -- Check the distance
  local torso_arm_ik = Kinematics.l_arm_torso( trLArm )
  
  local dist_pos = math.sqrt(
    (torso_arm_ik[1]-tr[1])^2+
    (torso_arm_ik[2]-tr[2])^2+
    (torso_arm_ik[3]-tr[3])^2)

  local dist_angle = math.sqrt(
    util.mod_angle( torso_arm_ik[4]-tr[4] )^2+
    util.mod_angle( torso_arm_ik[5]-tr[5])^2+
    util.mod_angle( torso_arm_ik[6]-tr[6])^2)

  if dist_pos>0.001 then return false end
  
  return trLArm, trLArm, dist_pos, dist_angle
end

-- take in a transform and output joint angles
Body.get_rarm_ik = function( tr )
  
  local qRArm = get_rarm_position();
  local trRArm = Kinematics.inverse_r_arm(tr, qRArm)
  
  -- Check the distance
  local torso_arm_ik = Kinematics.r_arm_torso( trRArm )
  
  local dist_pos = math.sqrt(
    (torso_arm_ik[1]-tr[1])^2+
    (torso_arm_ik[2]-tr[2])^2+
    (torso_arm_ik[3]-tr[3])^2)

  local dist_angle = math.sqrt(
    util.mod_angle( torso_arm_ik[4]-tr[4] )^2+
    util.mod_angle( torso_arm_ik[5]-tr[5])^2+
    util.mod_angle( torso_arm_ik[6]-tr[6])^2)

  if dist_pos>0.001 then return false end
  
  return trRArm, trRArm, dist_pos, dist_angle
end

return Body