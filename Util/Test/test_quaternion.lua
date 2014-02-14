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

angle0 = 45*DEG_TO_RAD
axis0 = {0,1,0}
angle1 = 45*DEG_TO_RAD
axis1 = {1,0,0}
t = 0
print(angle0*RAD_TO_DEG, unpack(axis0))
print(angle1*RAD_TO_DEG, unpack(axis1))
--
q0 = q.from_angle_axis(angle0,axis0)
q1 = q.from_angle_axis(angle1,axis1)
print(q0,q1)
qSlerp = q.slerp(q0,q1,t)
angle, axis = q.angle_axis( qSlerp )
print(angle*RAD_TO_DEG, unpack(axis))
