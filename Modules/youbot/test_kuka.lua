require'unix'
local youbot = require'youbot'

-- Init the base
youbot.init_base()
-- Init the arm
youbot.init_arm()
youbot.calibrate_gripper()

local steps1, steps2 = youbot.get_gripper_microsteps()
youbot.set_gripper_microsteps(1)
print('steps',steps1,steps2)
os.exit()

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
youbot.set_arm_pid1(1,20,0,0)
youbot.set_arm_pid2(1,20,0,0)
unix.usleep(1e6)
--]]
os.exit()

-- Read for 2 seconds
local t0 = unix.time()
local t = unix.time()
local rad = {}
while t-t0<1 do
  for i=1,5 do
    rad[i] = youbot.get_arm_position(i)
  end
  unix.usleep(1e4)
  t = unix.time()
end
print('Joints',unpack(rad))

-- Move the base arm
youbot.set_arm_angle(1,-2.9)
youbot.set_arm_angle(2,-1)
youbot.set_arm_angle(3,-2.5)
youbot.set_arm_angle(4,-1.7)
----[[
youbot.set_arm_angle(5,-2.95)
--]]
unix.usleep(3e6)

-- Read for 2 seconds
local t0 = unix.time()
local t = unix.time()
local rad = {}
while t-t0<1 do
  for i=1,5 do
    rad[i] = youbot.get_arm_position(i)
  end
  unix.usleep(1e4)
  t = unix.time()
end
print('Joints',unpack(rad))

-- Shutdown
youbot.shutdown_base()
youbot.shutdown_arm()

os.exit()
