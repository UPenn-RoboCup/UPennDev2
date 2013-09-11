dofile'../../include.lua'
os.execute('clear')

T = require'libTransform'
q = require'quaternion'
v = require'vector'
util = require'util'
-- Dipole components
--local r22 = math.sqrt(2)/2
x   = v.new{ .3, 0, .1 }
tri = v.new{ .3, .1, .2 }
-- form the dipole
dipole = tri-x
dipole_sz = v.norm(dipole)
dipole_torch = torch.Tensor(dipole)
dipole_norm = dipole / dipole_sz
dipole_norm_torch = torch.Tensor(dipole_norm)

-- Grab the various expressions of the dipole
T_dipole = T.from_dipole( dipole_norm_torch, x )
--util.ptorch( T_dipole )
q_dipole = q.from_dipole( dipole_norm )
--print('Quaternion',q_dipole)

-- Debug
print('Dipole',util.color(tostring(dipole),'blue'))
--[[
print'========'
print('Unit  ',util.color(tostring(dipole_norm),'green'))
--]]
--[[
print'========'
print('Dipole transform')
util.ptorch( T.position6D(T_dipole))
util.ptorch( T.to_rpy(T_dipole)*180/math.pi )
--]]
--[[
print'========'
print('q_dipole')
print('RPY degrees',q.to_rpy(q_dipole)*180/math.pi)
util.ptorch(T_q)
util.ptorch( T.to_rpy(T_q)*180/math.pi )
print'inverse'
util.ptorch(T.inv(T_q))
--]]

----------
-- Grip --
----------
-- User controlled variables
-- girth of the object/grip
local strata = .1 
-- from what side do we grip
local angle_of_attack = math.pi/6
-- percent along the object's axis
local climb = .5

-- Calculated values (for visualization, too)
cathode = T.from_quaternion(q_dipole,x)
print(util.color('cathode','magenta'))
util.ptorch( T.position6D(cathode))
--
anode = cathode*T.trans{0,0,dipole_sz}
print(util.color('anode','yellow'))
util.ptorch( T.position6D(anode))
--
centroid = cathode*T.trans{0,0,climb*dipole_sz}
print(util.color('centroid','green'))
util.ptorch(T.position6D(centroid))
-- 
orbit = cathode*T.rotZ(angle_of_attack)*T.trans{strata,0,climb*dipole_sz}
print(util.color('orbit','cyan'))
util.ptorch(T.position6D(orbit))