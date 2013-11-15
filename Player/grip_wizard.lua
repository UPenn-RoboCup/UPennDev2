dofile'include.lua'
require'jcm'
require'unix'
local Body = require'Body'
local lD = require'libDynamixel'
local usb2dyn = lD.new_bus('/dev/ttyUSB4',1e6)
local util = require'util'

--usb2dyn:ping_probe(1)
--os.exit()

local lclaw_id = 64
local lclaw_joint = Body.indexLGrip
--local rclaw_id = 65
local LOOP_RATE_HZ = 60
local LOOP_SEC = 1/60

local t0 = unix.time()
-- Loop forever
while true do
  local t = unix.time()

  -- Check if torque enabled or not
  if Body.get_lgrip_torque_enable(1)==1 then
    -- Left Command Position
    -- A command auto torque enables the motor
    local lclaw = Body.get_lgrip_command_position(1)
    local lstep = Body.make_joint_step(Body.indexLGrip,lclaw)
    lD.set_rx_command_position(lclaw_id,lstep,usb2dyn)
  else
    lD.set_rx_torque_enable(lclaw_id,0,usb2dyn)
  end

  -- Wait a millisecond for the motor to be ready for the read command
  unix.usleep(1e3)

  -- Read load/temperature/position/current
  local s, lall = lD.get_rx_everything(lclaw_id,usb2dyn)

  -- TODO: Put everything into shared memory
  if type(lall)=='table' then
    t_read = unix.time()
    jcm.sensorPtr.position[lclaw_joint] = 
      Body.make_joint_radian(lclaw_joint,lall.position)
    jcm.sensorPtr.velocity[lclaw_joint] = lall.speed
    jcm.sensorPtr.load[lclaw_joint] = lall.load
    jcm.sensorPtr.temperature[lclaw_joint] = lall.temperature
    -- time of Read
    jcm.treadPtr.position[lclaw_joint] = t_read
    jcm.treadPtr.velocity[lclaw_joint] = t_read
    jcm.treadPtr.load[lclaw_joint] = t_read
    jcm.treadPtr.temperature[lclaw_joint] = t_read
    --
    local t_read_diff = t_read - (t_read_last or t0)
    t_read_last = t_read
    print('Time diff:',t_read_diff)
    util.ptable(lall)
    print()
  elseif type(lall)=='number' then
    print('lall',lall)
  end

  -- Wait for the rate
  --local t_loop = unix.time()
  --local t_wait = 1e6*(LOOP_SEC - t_loop)
  unix.usleep(1e5)
end
