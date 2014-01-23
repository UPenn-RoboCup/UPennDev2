require'unix'
local youbot = require'youbot'

-- Init the base
youbot.init_base()
-- Init the arm
youbot.init_arm()

-- Commutation
unix.usleep(1e6)
print('Commutation!')
youbot.arm_commutation()
unix.usleep(1e6)

-- Get the limits
for i=1,5 do
  local lower, upper, en = youbot.get_arm_joint_limit(i)
  print('Joint '..i, lower, upper, en)
end

-- Read for 2 seconds
local t0 = unix.time()
local t = unix.time()
while t-t0<2 do
  print(t-t0)
  for i=1,5 do
    local rad = youbot.get_arm_position(i)
    print('Joint '..i, rad)
  end
  unix.usleep(1e6)
  t = unix.time()
end

-- Wait
--unix.usleep(1e6)
--youbot.calibrate_arm()

-- Move the base arm
youbot.set_arm_angle(1,2)
unix.usleep(3e6)

-- Shutdown
youbot.shutdown_base()
youbot.shutdown_arm()

os.exit()

-- Set the wheels to move forward
--youbot.set_base_velocity(0,0,0.1)

youbot.set_arm_angle(1,2)
youbot.set_arm_angle(2,2.8)
youbot.set_arm_angle(3,-2.43523)
youbot.set_arm_angle(4,1.73184)
youbot.set_arm_angle(5,0)
--unix.usleep(2e6)
