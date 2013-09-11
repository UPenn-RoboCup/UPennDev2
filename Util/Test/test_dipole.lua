dofile'../../include.lua'
os.execute('clear')

try_torch = true
if try_torch then
  T = require'libTransform'
else
  T = require'Transform'
end

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
dipole_norm = dipole / dipole_sz

-- Grab the various expressions of the dipole
q_dipole = q.from_dipole( dipole_norm )
--print('Quaternion',q_dipole)
if try_torch then
  dipole_torch = torch.Tensor(dipole)
  dipole_norm_torch = torch.Tensor(dipole_norm)
  T_dipole = T.from_dipole( dipole_norm_torch, x )
  --util.ptorch( T_dipole )
end

-- Debug
print('Dipole',util.color(tostring(dipole),'blue'),'@',x)
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
-- girth of the object/grip (>0)
local strata = .1 
-- from what side do we grip (mod_angle)
local angle_of_attack = math.pi/6
-- percent along the object's axis
-- (+1 and <0 are allowed, to an extent...)
local climb = .5

-- Calculated values (for visualization, too)
cathode = T.from_quaternion(q_dipole,x)
print(util.color('cathode','magenta')) -- x
util.ptorch( T.position6D(cathode))
--
anode = cathode*T.trans(0,0,dipole_sz)
print(util.color('anode','yellow')) -- triangle
util.ptorch( T.position6D(anode))
--
centroid = cathode*T.trans(0,0,climb*dipole_sz)
print(util.color('centroid','green')) -- box
util.ptorch(T.position6D(centroid))
-- 
orbit = cathode*T.rotZ(angle_of_attack)*T.trans(strata,0,climb*dipole_sz)
print(util.color('orbit','cyan')) -- circle
util.ptorch(T.position6D(orbit))

--------
-- With overall robot
--------
-- Torso rotation
torso = T.rotX(10*math.pi/180)*T.rotY(15*math.pi/180)*T.rotZ(5*math.pi/180)
--print(torso)
-- Global pose
local pose = v.pose{15,25,43*math.pi/180}
body = T.trans(pose.x,pose.y,0)*T.rotZ(pose.a)
-- Overall robot
robot = body * torso
print(util.color('robot','red'))
util.ptorch(T.position6D(robot))