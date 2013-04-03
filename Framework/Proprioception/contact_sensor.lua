require('dcm')
require('pcm')
require('wrench')
require('Config')

--------------------------------------------------------------------------------
-- contact_sensor : estimates end-effector wrenches from F/T readings
--------------------------------------------------------------------------------

contact_sensor = {}

local force_torque = Config.force_torque

local l_foot = Config.mechanics.l_foot
local r_foot = Config.mechanics.r_foot
local l_hand = Config.mechanics.l_hand
local r_hand = Config.mechanics.r_hand
local l_foot_wrench_offset = -l_foot.force_torque_transform:get_pose()
local r_foot_wrench_offset = -r_foot.force_torque_transform:get_pose()
local l_hand_wrench_offset = -l_hand.force_torque_transform:get_pose()
local r_hand_wrench_offset = -r_hand.force_torque_transform:get_pose()

function contact_sensor.entry()
end

function contact_sensor.update()
  local l_foot_wrench = wrench.new(dcm:get_force_torque(force_torque.l_foot))
  local r_foot_wrench = wrench.new(dcm:get_force_torque(force_torque.r_foot))
  local l_hand_wrench = wrench.new(dcm:get_force_torque(force_torque.l_hand))
  local r_hand_wrench = wrench.new(dcm:get_force_torque(force_torque.r_hand))

  -- update pcm
  pcm:set_l_foot_wrench(l_foot_wrench:translate(l_foot_wrench_offset))
  pcm:set_r_foot_wrench(r_foot_wrench:translate(r_foot_wrench_offset))
  pcm:set_l_hand_wrench(l_hand_wrench:translate(l_hand_wrench_offset))
  pcm:set_r_hand_wrench(r_hand_wrench:translate(r_hand_wrench_offset))
end

function contact_sensor.exit()
end

return contact_sensor
