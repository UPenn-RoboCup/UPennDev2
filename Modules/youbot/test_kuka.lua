require'unix'
local youbot = require'youbot'

-- Init the base
youbot.init_base()

os.exit()

-- Set the wheels to move forward
--youbot.set_base_velocity(0,0,0.1)

--Init the arm
youbot.init_arm()

youbot.calibrate_arm()

os.exit()

youbot.set_arm_angle(1,2)
youbot.set_arm_angle(2,2.8)
youbot.set_arm_angle(3,-2.43523)
youbot.set_arm_angle(4,1.73184)
youbot.set_arm_angle(5,0)
--unix.usleep(2e6)

while true do
for i=1,5 do
local rad = youbot.get_arm_position(i)
print('Joint '..i, rad)
end
unix.usleep(1e6)
end

youbot.shutdown_arm()
