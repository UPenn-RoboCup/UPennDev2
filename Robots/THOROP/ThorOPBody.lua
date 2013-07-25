-- THOR OP Kinematics
-- Dynamixel definitions
local libDynamixel = require'libDynamixel2'

-- Shared memory for the joints
require'jcm'

-- Utilities
require'vector'

local DEG_TO_RAD = math.pi/180

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
--]]
assert(nJoint==(indexAux+nJointAux)-indexHead,'Bad joint counting!')
Body.nJoint = nJoint
Body.parts = {
  ['Head']=vector.count(indexHead,nJointHead),
  ['Spine']=vector.count(indexSpine,nJointSpine),
  ['LArm']=vector.count(indexLArm,nJointLArm),
  ['LLeg']=vector.count(indexLLeg,nJointLLeg),
  ['RLeg']=vector.count(indexRLeg,nJointRLeg),
  ['RArm']=vector.count(indexRArm,nJointRArm),
  ['Aux']=vector.count(indexAux,nJointAux)
}

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
  1,1,1, -- left gripper
  1,1,1, -- right gripper
  1, -- Lidar pan
  
  })
assert(#servo.direction==nJoint,'Bad servo direction!')

-- TODO: Use some loop based upon MX/NX
servo.moveRange = 360 * DEG_TO_RAD * vector.ones(nJoint)
servo.steps = 4096 * vector.ones(nJoint)
Body.servo = servo

--------------------------------
-- Standard convenience functions to access jcm
for part,list in pairs(Body.parts) do
  Body['get_'..part:lower()..'_position'] = function(idx)
    if idx then return jcm.sensorPtr.position[ list[idx] ] end
    return jcm.sensorPtr.position:table( list[1], list[#list] )
  end -- Get
  Body['set_'..part:lower()..'_command'] = function(val,idx)
    if type(val)=='number' then
      if idx then jcm.actuatorPtr.command[ list[idx] ] = val; return; end
      for _,l in ipairs(list) do
        jcm.actuatorPtr.command[l] = val
      end
    else
      for i,l in ipairs(list) do
        jcm.actuatorPtr.command[l] = val[i]
      end
    end -- if number
  end -- Set
end

--------------------------------
-- Additional Convenience functions to access jcm
for part,list in pairs(Body.parts) do
  Body['get_'..part:lower()..'_command'] = function(idx)
    if idx then return jcm.actuatorPtr.command[ list[idx] ] end
    return jcm.actuatorPtr.command:table( list[1], list[#list] )
  end -- Get
  Body['set_'..part:lower()..'_position'] = function(val,idx)
    if type(val)=='number' then
      if idx then jcm.sensorPtr.position[ list[idx] ] = val; return; end
      for _,l in ipairs(list) do
        jcm.sensorPtr.position[l] = val
      end
    else
      for i,l in ipairs(list) do
        jcm.sensorPtr.position[l] = val[i]
      end
    end -- if number
  end -- Set
end

--[[
-- Get
function get_head_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexHead, indexHead+nJointHead-1)}
end
function get_spine_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexSpine, indexSpine+nJointSpine-1)}
end
function get_larm_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexLArm, indexLArm+nJointLArm-1)}
end
function get_rarm_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexRArm, indexRArm+nJointRArm-1)}
end
function get_lleg_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexLLeg, indexLLeg+nJointLLeg-1)}
end
function get_rleg_position()
  local q = jcm.get_sensor_position()
  return {unpack(q, indexRLeg, indexRLeg+nJointRLeg-1)}
end
-- Set
function set_head_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexHead] = val
  end
end
function set_spine_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexSpine] = val
  end
end
function set_lleg_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexLLeg] = val
  end
end
function set_rleg_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexRLeg] = val
  end
end
function set_larm_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexLArm] = val
  end
end
function set_rarm_command(val)
  for i=indexHead,#val do
    jcm.actuatorPtr.command[indexRArm] = val
  end
end
--]]


--------------------------------
-- Convenience functions
-- Use libDynamixel to have access to more settings, via instructions
----[[
for k,v in pairs(libDynamixel.nx_registers) do
  local get_func = libDynamixel['get_nx_'..k]
  local set_func = libDynamixel['get_nx_'..k]
  for part,list in pairs(Body.parts) do
    Body['get_'..part..'_'..k..'_packet'] = function()
      return get_func(list)
    end
    Body['set_'..part..'_'..k..'_packet'] = function( vals )
      return set_func(list,vals)
    end
  end
end
-- Gripper is MX
-- TODO: make general somehow
for k,v in pairs(libDynamixel.mx_registers) do
  local get_func = libDynamixel['get_mx_'..k]
  local set_func = libDynamixel['get_mx_'..k]
  Body['get_aux_'..k..'_packet'] = function()
    return get_func(list)
  end
  Body['set_aux_'..k..'_packet'] = function( vals )
    return set_func(list,vals)
  end
end
--]]

return Body