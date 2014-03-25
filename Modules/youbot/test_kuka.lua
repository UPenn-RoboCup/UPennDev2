require'unix'
local youbot = require'youbot'

-- Init the base
youbot.init_base()
-- Init the arm
youbot.init_arm()
--youbot.calibrate_gripper()

-- Get the limits
for j=1,5 do
  print('Joint',j)
  local lower, upper, en = youbot.get_arm_joint_limit(j)
	print("Limits",lower, upper, en)
  local p,i,d = youbot.get_arm_pid1(j)
	print("PID1",p,i,d)
  local p,i,d = youbot.get_arm_pid2(j)
	print("PID2",p,i,d)
	print()
end
----[[
youbot.set_arm_pid1(1,1,0,0)
youbot.set_arm_pid2(1,2,0,0)
unix.usleep(1e6)
--]]

-- Read for 2 seconds
local t0 = unix.time()
local t = unix.time()
local rad = {}
while t-t0<5 do
  for i=1,5 do
    rad[i] = youbot.get_arm_position(i)
    print('Joint '..i, rad[i])
  end
  unix.usleep(1e4)
  t = unix.time()
end


-- Move the base arm
youbot.set_arm_angle(1,-.1)
unix.usleep(3e6)

-- Shutdown
youbot.shutdown_base()
youbot.shutdown_arm()

os.exit()
