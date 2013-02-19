require('dcm')
require('pcm')
require('Config')

--------------------------------------------------------------------------------
-- contact_sensor : estimates end-effector wrenches from F/T readings
--------------------------------------------------------------------------------

contact_sensor = {}

local l_foot = Config.mechanics.l_foot
local r_foot = Config.mechanics.r_foot
local l_hand = Config.mechanics.l_hand
local r_hand = Config.mechanics.r_hand
local l_foot_wrench_offset = -l_foot.force_torque_transform:get_pose6D()
local r_foot_wrench_offset = -r_foot.force_torque_transform:get_pose6D()
local l_hand_wrench_offset = -l_hand.force_torque_transform:get_pose6D()
local r_hand_wrench_offset = -r_hand.force_torque_transform:get_pose6D()

local function translate_wrench(w, offset)
  -- compute equivalent wrench at desired offset using sum of moments
  local fx, fy, fz = w[1], w[2], w[3]
  local tx, ty, tz = w[4], w[5], w[6]
  local rx, ry, rz = offset[1], offset[2], offset[3]
  w[4] = w[4] - fz*ry + fy*rz
  w[5] = w[5] - fx*rz + fz*rx 
  w[6] = w[6] - fy*rx + fx*ry
end

function contact_sensor.entry()
end

function contact_sensor.update()
  local l_foot_wrench = dcm:get_force_torque('l_foot')
  local r_foot_wrench = dcm:get_force_torque('r_foot')
  local l_hand_wrench = dcm:get_force_torque('l_hand')
  local r_hand_wrench = dcm:get_force_torque('r_hand')
  translate_wrench(l_foot_wrench, l_foot_wrench_offset)
  translate_wrench(r_foot_wrench, r_foot_wrench_offset)
  translate_wrench(l_hand_wrench, l_hand_wrench_offset)
  translate_wrench(r_hand_wrench, r_hand_wrench_offset)

  -- update pcm
  pcm:set_l_foot_wrench(l_foot_wrench)
  pcm:set_r_foot_wrench(r_foot_wrench)
  pcm:set_l_hand_wrench(l_hand_wrench)
  pcm:set_r_hand_wrench(r_hand_wrench)
end

function contact_sensor.exit()
end

return contact_sensor
