dofile('../../include.lua')

require('Dynamics')
require('Transform')
require('vector')
require('unix')

-- define leg joint positions and velocities
local q = vector.zeros(27)
local qdot = vector.zeros(27)
local qdotdot = vector.zeros(27)

q[3] =-math.pi/4
q[4] = math.pi/2
q[5] =-math.pi/4

q[9] =-math.pi/4
q[10] = math.pi/2
q[11] =-math.pi/4

local torso_pose = vector.zeros(6)
local torso_twist = vector.zeros(6)
local torso_accel = vector.zeros(6)

local l_foot_wrench = vector.new{0, 0, 0, 0, 0, 0}
local r_foot_wrench = vector.new{0, 0, 0, 0, 0, 0}
local l_hand_wrench = vector.new{0, 0, 0, 0, 0, 0}
local r_hand_wrench = vector.new{0, 0, 0, 0, 0, 0}

print('q', q)
print('qdot', qdot)
print('qdotdot', qdotdot)
print('torso_pose', torso_pose)
print('torso_twist', torso_twist)
print('torso_accel', torso_accel)
print('l_foot_wrench', l_foot_wrench)
print('r_foot_wrench', r_foot_wrench)
print('l_hand_wrench', l_hand_wrench)
print('r_hand_wrench', r_hand_wrench)

-- calculate inverse dynamics 
local torques, floating_base_torques = Dynamics.inverse(
  q,
  qdot,
  qdotdot,
  torso_pose,
  torso_twist,
  torso_accel,
  l_foot_wrench,
  r_foot_wrench,
  l_hand_wrench,
  r_hand_wrench
)

print('floating base torques :', floating_base_torques)
print('joint torques :', torques)
