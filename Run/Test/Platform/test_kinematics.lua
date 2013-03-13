dofile('../../include.lua')

require('Kinematics')
require('Transform')
require('vector')
require('unix')

-- define leg joint positions and velocities
local q = vector.zeros(12)

q[1] = -1.0
q[2] = 0
q[3] = 0
q[4] = 0
q[5] = 0
q[6] = 0

q[7] = -1.0
q[8] = 0
q[9] = 0
q[10] = 0
q[11] = 0
q[12] = 0

local qdot = vector.zeros(12)

qdot[1] = 0 
qdot[2] = 0
qdot[3] = 5 
qdot[4] = -5 
qdot[5] = 0
qdot[6] = 0

qdot[7] = 0 
qdot[8] = 0
qdot[9] = 5 
qdot[10] = -5 
qdot[11] = 0
qdot[12] = 0

print('q', q )
print('qdot', qdot )

-- calculate forward kinematics 
t0 = unix.time()
local p_left, p_right, v_left, v_right 
  = Kinematics.forward_vel_legs(q, qdot)

print('\n\nforward kinematics time: ', unix.time() - t0)
print('p_left')
print( p_left )
print('p_right')
print( p_right )
print('v_left', v_left )
print('v_right', v_right )

-- calculate inverse kinematics 
t0 = unix.time()
local q_inverse, qdot_inverse =
  Kinematics.inverse_vel_legs(p_left, p_right, v_left, v_right)

print('\n\ninverse kinematics time: ', unix.time() - t0)
print('q_inverse', q_inverse )
print('qdot_inverse', qdot_inverse )
