dofile'../../include.lua'
local vector = require'vector'
local K = require'THOROPKinematics'
local T = require'Transform'
local K2 = require'fk2'
local torch = require'torch'
local util = require'util'

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
local qLArm = vector.new({0,0,0, 0, 0,0,0})
local qLArm2 = vector.new({90,0,0, -90, 0,0,0})*DEG_TO_RAD
-- Find the forward kinematics
local fL = K.l_arm_torso_7(qLArm, 0, {0,0}, 0,0,0)
local fL2 = K.l_arm_torso_7(qLArm2, 0, {0,0}, 0,0,0)
fL[2] = fL[2] - K.shoulderOffsetY
fL[3] = fL[3] - K.shoulderOffsetZ
fL2[2] = fL2[2] - K.shoulderOffsetY
fL2[3] = fL2[3] - K.shoulderOffsetZ
print('Left Angles', qLArm)
print('Left end effector x,y,z, roll, pitch, yaw')
print()
print(vector.new(fL))
print(vector.new(fL2))

-- Test other FK
print()
print('=======NewK=========')
print()
local fL2 = K2.fk(qLArm)
local fL2a = K2.fk(qLArm2)
print(T.position6D(fL2))
print(T.position6D(fL2a))
--util.ptorch(fL2)
--util.ptorch(fL2a)