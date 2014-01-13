require'unix'
local kuka = require'kuka'

-- Init the base
--kuka.init_base()
-- Set the wheels to move forward
--kuka.set_base_velocity(0,0,0.1)

--Init the arm
kuka.init_arm()

--kuka.calibrate_arm()

kuka.set_arm_angle(1,2)
kuka.set_arm_angle(2,2.8)
kuka.set_arm_angle(3,-2.43523)
kuka.set_arm_angle(4,1.73184)
kuka.set_arm_angle(5,0)
--unix.usleep(2e6)

while true do
for i=1,5 do
local rad = kuka.get_arm_position(i)
print('Joint '..i, rad)
end
unix.usleep(1e6)
end

kuka.shutdown_arm()
