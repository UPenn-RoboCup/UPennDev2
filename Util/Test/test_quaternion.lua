dofile'../../include.lua'
os.execute('clear')

--try_torch = true
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

print('=======')
function randAA()
	local ang = math.random()*360*DEG_TO_RAD-180*DEG_TO_RAD
	local axis = v.new{
		math.random(),
		math.random(),
		math.random()
	}
	return ang, axis / v.norm(axis)
end
local quats = {}
for i=1,100 do
	local ang, axis = randAA()
	print(ang, axis)
	quats[i] = q.from_angle_axis(ang, axis)
end

local fromQ = require'Transform'.from_quatp
local toQ = require'Transform'.to_quatp

local quatps = {}
for i,quat in ipairs(quats) do
	local quatp = {unpack(quat)}
	quatp[5] = math.random() * 10
	quatp[6] = math.random() * 10
	quatp[7] = math.random() * 10
	quatps[i] = v.new(quatp)
end

for i,quatp in ipairs(quatps) do
	print('*')
	print(quatp)
	local tr = fromQ(quatp)
	print(tr)
	local quatp1 = toQ(tr)
	v.new(quatp1)
	--local tr1 = fromQ(quatp1)
	print(quatp, quatp1)
	--print(quatp1-quatp)
end
