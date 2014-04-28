bhwalk = require'bhwalk'

for k, v in pairs(bhwalk) do
  print(k,v)
end

-- Try to walk
local nJoint = 22
local jAngles, jCurrents = {}, {}
for i=1,nJoint do
  jAngles[i] = 0
  jCurrents[i] = 0
end
bhwalk.walk_request(10,10,0)
for i=1,5 do
  bhwalk.set_sensor_angles(jAngles)
  bhwalk.set_sensor_currents(jCurrents)
  bhwalk.update()
  print("Joints", unpack(bhwalk.get_joint_angles()) )
  print("Odometry", unpack(bhwalk.get_odometry()) )
end
