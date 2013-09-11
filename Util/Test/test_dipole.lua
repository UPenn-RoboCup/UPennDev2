dofile'../../include.lua'
T = require'libTransform'
q = require'quaternion'
v = require'vector'
util = require'util'
-- Dipole components
x   = v.new{ 0.3, 0.0, 0.1}
tri = v.new{ 0.4, 0.1, 0.1}
-- form the dipole
dipole = tri-x
dipole_norm = dipole / v.norm(dipole)
dipole_norm_torch = torch.Tensor(dipole)

-- Grab the various expressions of the dipole
T_dipole = T.from_dipole( dipole_norm_torch, x )
q_dipole = q.from_dipole( dipole_norm )

-- Debug
os.execute('clear')
print'========'
print('Dipole',util.color(tostring(dipole),'green'))
print('Unit  ',util.color(tostring(dipole_norm),'green'))
print'========'
print('Dipole transform')
util.ptorch( T_dipole )
util.ptorch( T.position6D(T_dipole))
print'========'
print('q_dipole')
print('Quaternion',q_dipole)
print('RPY degrees',q.to_rpy(q_dipole)*180/math.pi)
util.ptorch(T.from_quaternion(q_dipole,x))