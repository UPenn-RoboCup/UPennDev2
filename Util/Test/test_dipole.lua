dofile'../../include.lua'
os.execute('clear')

T = require'libTransform'
q = require'quaternion'
v = require'vector'
util = require'util'
-- Dipole components
--local r22 = math.sqrt(2)/2
x   = v.new{ 0, 0, 0}
tri = v.new{ 0, 1, 1}
-- form the dipole
dipole = tri-x
dipole_norm = dipole / v.norm(dipole)
dipole_norm_torch = torch.Tensor(dipole_norm)

-- Grab the various expressions of the dipole
T_dipole = T.from_dipole( dipole_norm_torch, x )
q_dipole = q.from_dipole( dipole_norm )

-- Debug
print'========'
print('Dipole',util.color(tostring(dipole),'green'))
print('Unit  ',util.color(tostring(dipole_norm),'green'))
----[[
print'========'
print('Dipole transform')
util.ptorch( T_dipole )
util.ptorch( T.position6D(T_dipole))
util.ptorch( T.to_rpy(T_dipole)*180/math.pi )
--]]
print'========'
print('q_dipole')
print('Quaternion',q_dipole)
print('RPY degrees',q.to_rpy(q_dipole)*180/math.pi)
local T_q = T.from_quaternion(q_dipole,x)
util.ptorch(T_q)
util.ptorch( T.position6D(T_q))
util.ptorch( T.to_rpy(T_q)*180/math.pi )