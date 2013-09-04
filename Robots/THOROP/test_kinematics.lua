dofile'../../include.lua'
local vector = require'vector'
local K = require'THOROPKinematics'
local T = require'Transform'

print()
print('================')
print()

print'Kinematics Functions and variables'
for k,v in pairs(K) do print(k,v) end

print()
print('================')
print()

-- Test forward left
-- 6 DOF arm, with angles given in qLArm
local qLArm = vector.new({0,0,0, 0,0,0})
-- Find the forward kinematics
local fL = K.forward_larm(qLArm)
print('Left Angles',qLArm)
print('Left end effector x,y,z, roll, pitch, yaw', T.position6D(fL) )