dofile('../include.lua')

require('pcm')
require('dcm')
require('rpc')
require('unix')
require('util')
require('curses')
require('Config')
require('Platform')
require('Proprioception')
require('Dynamics')
require('Transform')

-- initialize state
Platform.entry()
Proprioception.entry()

-- set position control
local q0 = {0, 0, -0.2, 0.4, -0.2, 0, 0, 0, -0.2, 0.4, -0.2, 0}
dcm:set_joint_position(q0, 'legs')
dcm:set_joint_velocity(0, 'all')
dcm:set_joint_stiffness(1, 'all')
dcm:set_joint_damping(0, 'all')

local l_foot_wrench = pcm:get_l_foot_wrench()
local r_foot_wrench = pcm:get_r_foot_wrench()
local l_hand_wrench = vector.zeros(6) 
local r_hand_wrench = vector.zeros(6)

for i = 1,100 do
  Platform.update()
  Proprioception.update()
  l_foot_wrench = (l_foot_wrench + pcm:get_l_foot_wrench())/2
  r_foot_wrench = (r_foot_wrench + pcm:get_r_foot_wrench())/2
end

-- set force control
local q = dcm:get_joint_position()
local qdot = dcm:get_joint_velocity()
local qdotdot = vector.zeros(#qdot)
local torso_frame = Transform.pose(pcm:get_torso_pose())
local torso_twist = pcm:get_torso_twist()

local tau, torso_accel = Dynamics.inverse(
  q,
  qdot,
  qdotdot,
  torso_pose,
  torso_twist,
  l_foot_wrench,
  r_foot_wrench,
  l_hand_wrench,
  r_hand_wrench
);

dcm:set_joint_force(1*tau)
dcm:set_joint_stiffness(0.005, 'all')

while true do
  Platform.update()
  Proprioception.update()
end

Platform.exit()
Proprioception.exit()
