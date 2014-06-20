---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil
if not IS_WEBOTS then
  signal = require'signal'
  function shutdown ()
    running = false
    --os.exit()
  end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 1.0, t0

-- Entry
Body.entry()
-- Update loop
while running do
  t = get_time()
  -- Update the body
  Body.update()
  -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    os.execute('clear')
		print(string.format('Body | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))

    local larm_msg = mcm.get_servo_larm()
    local rarm_msg = mcm.get_servo_rarm()
    local lleg_msg = mcm.get_servo_lleg()
    local rleg_msg = mcm.get_servo_rleg()
    local imu_msg = mcm.get_servo_imu()

    print(string.format('LArm | Uptime %.2f sec, Mem %d kB, %.2f Hz',unpack(larm_msg)))
    print(string.format('RArm | Uptime %.2f sec, Mem %d kB, %.2f Hz',unpack(rarm_msg)))
    print(string.format('LLeg | Uptime %.2f sec, Mem %d kB, %.2f Hz',unpack(lleg_msg)))
    print(string.format('RLeg | Uptime %.2f sec, Mem %d kB, %.2f Hz',unpack(rleg_msg)))


    print(string.format('IMU | Uptime %.2f sec, Mem %d kB, %.2f Hz\n'..
                        'Acc: %.2f %.2f %.2f\nGyro: %.3f,%.3f,%.3f\n'..
                        'Mag: %.2f %.2f %.2f\nRPY (deg): %.2f,%.2f,%.2f (%.2f)\n',
      imu_msg[1],imu_msg[2],imu_msg[3],
      imu_msg[4],imu_msg[5],imu_msg[6],
      imu_msg[7],imu_msg[8],imu_msg[9],
      imu_msg[10],imu_msg[11],imu_msg[12],
      imu_msg[13]*180/math.pi,imu_msg[14]*180/math.pi,imu_msg[15]*180/math.pi,
      imu_msg[16]*180/math.pi

))


    --print('Wire', vcm.get_wire_model())
	end
end

-- Exit
print'Exiting body wizard...'
Body.exit()
os.exit()
