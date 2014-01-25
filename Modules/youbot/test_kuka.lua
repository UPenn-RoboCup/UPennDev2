require'unix'
local youbot = require'youbot'

-- Init the base
youbot.init_base()
-- Init the arm
youbot.init_arm()
youbot.calibrate_gripper()

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


-- Move the base arm
--youbot.set_arm_angle(1,2)
--unix.usleep(3e6)

-- Shutdown
youbot.shutdown_base()
youbot.shutdown_arm()

os.exit()
