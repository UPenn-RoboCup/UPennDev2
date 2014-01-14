dofile'../../include.lua'
local vector = require'vector'
local K = require'YouBotKinematics'
local T = require'Transform'

print('\n================')
print'\nKinematics Functions and variables'
for k,v in pairs(K) do print(k,v) end
print('\n================')

-- Test forward straight up
print('\n+z | Arm Straight Up')
-- 6 DOF arm, with angles given in qLArm
local qArm = vector.new{0,0,0, 0,0,0}
-- Find the forward kinematics
local fA = K.forward_arm(qArm)
print('+z | Arm Angles:\n',qArm)
print('+z | Arm end effector x,y,z, roll, pitch, yaw:\n', T.position6D(fA) )
local iqArm = vector.new( K.inverse_arm(T.position6D(fA)) )
print('+z | Inverse Angles:\n',iqArm)

-- Test forward a bit out
print('\n+x | Arm Straight Out forward')
-- 6 DOF arm, with angles given in qLArm
local qArm = vector.new{0,math.pi/2,0, 0,0,0}
-- Find the forward kinematics
local fA = K.forward_arm(qArm)
print('+x | Arm Angles:\n',qArm)
print('+x | Arm end effector x,y,z, roll, pitch, yaw:\n', T.position6D(fA) )
local iqArm = vector.new( K.inverse_arm(T.position6D(fA)) )
print('+x | Inverse Angles:\n',iqArm)

-- Test yaw & forward a bit out
print('\n+x+y | Arm Straight Out and Yaw')
-- 6 DOF arm, with angles given in qLArm
local qArm = vector.new{math.pi/4,math.pi/2,0, 0,0,0}
-- Find the forward kinematics
local fA = K.forward_arm(qArm)
print('+x+y | Arm Angles:\n',qArm)
print('+x+y | Arm end effector x,y,z, roll, pitch, yaw:\n', T.position6D(fA) )
local iqArm = vector.new( K.inverse_arm(T.position6D(fA)) )
print('+x+y | Inverse Angles:\n',iqArm)

-- Test inverse somewhat straight up
print('\n+x | Arm Up slack')
local pArm = {0,0,.5, 0,0,0}
local iqArm = vector.new( K.inverse_arm(pArm) )
print('+x | Inverse Angles:\n',iqArm)
local fA = K.forward_arm(iqArm)
print('+x+y | Arm end effector x,y,z, roll, pitch, yaw:\n', T.position6D(fA) )