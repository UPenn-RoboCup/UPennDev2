dofile'../../include.lua'
local vector = require'vector'
local K = require'THOROPKinematics'
local T = require'Transform'
local K2 = require'fk2'
local torch = require'torch'
local util = require'util'
local ok, ffi = pcall(require, 'ffi')

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
local qLArm = vector.new({0,0,0, -1, 0,0,0})*DEG_TO_RAD
local qLArm2 = vector.new({90,0,0, -45, 0,0,0})*DEG_TO_RAD
local fL2_t, fL2a_t = torch.eye(4), torch.eye(4)
-- Hack to make super fast (don't go to C ever)
if jit then
	fL2_d = fL2_t:data()
	fL2a_d = fL2a_t:data()
end

dt_all = vector.zeros(6)
local n = 1000
for i=1,n do
	t0 = unix.time()
	fL = K.l_arm_torso_7(qLArm, 0, {0,0}, 0,0,0)
	t1 = unix.time()
	fL2 = K2.fk(qLArm)
	t2 = unix.time()
	if fL2_d then K2.fk_t(fL2_d, qLArm) end
	t3 = unix.time()
	fLa = K.l_arm_torso_7(qLArm2, 0, {0,0}, 0,0,0)
	t4 = unix.time()
	fL2a = K2.fk(qLArm2)
	t5 = unix.time()
	if fL2a_d then K2.fk_t(fL2a_d, qLArm2) end
	t6 = unix.time()
	dt = vector.new{t1-t0, t2-t1,t3-t2,t4-t3,t5-t4,t6-t5}
	dt_all = dt_all + dt
end
print('Times:',dt_all)

-- Find the forward kinematics
fL[2] = fL[2] - K.shoulderOffsetY
fL[3] = fL[3] - K.shoulderOffsetZ
fLa[2] = fLa[2] - K.shoulderOffsetY
fLa[3] = fLa[3] - K.shoulderOffsetZ

--print('Left end effector x,y,z, roll, pitch, yaw')
--if ffi then ffi.new('double[4][4]',fL2) end

-- Test other FK
print()
print('Left Angles', qLArm)
print(vector.new(fL))
print(T.position6D(fL2))
print(T.position6D(fL2_t))
print()
print('Left Angles', qLArm2)
print(vector.new(fLa))
print(T.position6D(fL2a))
print(T.position6D(fL2a_t))

print()
print('IK2')
local ik2 = require'ik2'

fL[2] = fL[2] + K.shoulderOffsetY
fL[3] = fL[3] + K.shoulderOffsetZ
fLa[2] = fLa[2] + K.shoulderOffsetY
fLa[3] = fLa[3] + K.shoulderOffsetZ

fL2_t4 = ffi.new('double[4][4]', fL2)
fL2a_t4 = ffi.new('double[4][4]', fL2a)

dt_all = vector.zeros(3)
--n = 10
for i=1,n do
	t0 = unix.time()
	iqLArm = K.inverse_l_arm_7(fL, qLArm, 0, 0, {0,0}, 0,0,0, 0)
	t1 = unix.time()
	iqLArm2 = ik2.ik(fL2_t, qLArm, 0, false)
	t2 = unix.time()
	iqLArm3 = ik2.ik2(fL2_t4, qLArm, 0, false)
	t3 = unix.time()
	dt_all = vector.new{t1-t0, t2-t1, t3-t2} + dt_all
end

iqLArm_a = K.inverse_l_arm_7(fLa, qLArm2, 0, 0, {0,0}, 0,0,0, 0)
iqLArm2a = ik2.ik(fL2a_t, qLArm2, 0, false)
iqLArm3a = ik2.ik2(fL2a_t4, qLArm2, 0, false)

print('Time:', dt_all)
print()
print(qLArm)
print(vector.new(iqLArm))
print(vector.new(iqLArm2))
print(vector.new(iqLArm3))
print()
print(qLArm2)
print(vector.new(iqLArm_a))
print(vector.new(iqLArm2a))
print(vector.new(iqLArm3a))

--[[
K2 = require'K_ffi'
T = require'libTransform'

q = Body.get_rarm_position()
tr = K2.forward_r_arm(q)
iq = K2.inverse_r_arm(tr, q)
itr = K2.forward_r_arm(iq)
Body.set_rarm_command_position(iq)

q = Body.get_rarm_position()
tr1 = T.copy(itr)
iq1 = K2.inverse_r_arm(tr1, q, -5*DEG_TO_RAD)
itr1 = K2.forward_r_arm(iq1)
Body.set_rarm_command_position(iq1)

q = Body.get_rarm_position()
tr2 = T.trans(0.4,0,0.4) * tr
iq2 = K2.inverse_r_arm(tr2, q, -20*DEG_TO_RAD)
itr2 = K2.forward_r_arm(iq2)
Body.set_rarm_command_position(iq2)

q = Body.get_rarm_position()
tr = K2.forward_r_arm(q)
tr3 = tr * T.rotY(-15*DEG_TO_RAD)
iq3 = K2.inverse_r_arm(tr3, q)
itr3 = K2.forward_r_arm(iq3)
Body.set_rarm_command_position(iq3)

q = Body.get_rarm_position()
tr = K2.forward_r_arm(q)
tr4 = tr * T.trans(0,0.1,0)
iq4 = K2.inverse_r_arm(tr4, q)
itr4 = K2.forward_r_arm(iq4)
Body.set_rarm_command_position(iq4)

--------

q = Body.get_larm_position()
tr = K2.forward_l_arm(q)
iq = K2.inverse_l_arm(tr, q)
itr = K2.forward_l_arm(iq)
Body.set_larm_command_position(iq)

q = Body.get_larm_position()
tr1 = T.copy(itr)
iq1 = K2.inverse_l_arm(tr1, q, 5*DEG_TO_RAD)
itr1 = K2.forward_l_arm(iq1)
Body.set_larm_command_position(iq1)

q = Body.get_larm_position()
tr2 = T.trans(0.2,0.2,-0.1) * tr
iq2 = K2.inverse_l_arm(tr2, q, 20*DEG_TO_RAD)
itr2 = K2.forward_l_arm(iq2)
Body.set_larm_command_position(iq2)

q = Body.get_larm_position()
tr = K2.forward_l_arm(q)
tr3 = tr * T.rotY(-15*DEG_TO_RAD)
iq3 = K2.inverse_l_arm(tr3, q)
itr3 = K2.forward_l_arm(iq3)
Body.set_larm_command_position(iq3)

q = Body.get_larm_position()
tr = K2.forward_l_arm(q)
tr4 = tr * T.trans(0,0.1,0)
iq4 = K2.inverse_l_arm(tr4, q)
itr4 = K2.forward_l_arm(iq4)
Body.set_larm_command_position(iq4)

P = require'libPlan'
K = require'K_ffi'
T = require'libTransform'
q = Body.get_larm_position()
tr = K.forward_l_arm(q)
tr1 = tr*T.trans(0.05,0,0)
pl=P.new_planner(K)
it = pl:line_iter(tr1, q)
=it(q)
=q
--]]

--[[
T = require'Transform'
K = require'K_ffi'
qR = vector.zeros(7)
sh = -30*DEG_TO_RAD
trR = T.transform6D({0.2, -0.3, 0.13, 0, 0*DEG_TO_RAD, 45*DEG_TO_RAD})
iqR = K.inverse_r_arm(trR, qR, sh)
itrR = K.forward_r_arm(iqR)
print(trR)
print(itrR)
print(qR)
print(iqR)
--]]

--[[
T = require'Transform'
K = require'K_ffi'
qL = vector.zeros(7)
sh = 30*DEG_TO_RAD
trL = T.transform6D({0.22, 0.3, 0.13, 0, 0*DEG_TO_RAD, 45*DEG_TO_RAD})
iqL = K.inverse_l_arm(trL, qL, sh)
itrL = K.forward_l_arm(iqL)
print(trL)
print(itrL)
print(qL)
print(iqL)
--]]