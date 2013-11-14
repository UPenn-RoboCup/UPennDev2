dofile'include.lua'
require'jcm'
require'unix'
local Body = require'Body'
local lD = require'libDynamixel'
local usb2dyn = lD.new_bus('/dev/ttyUSB4',1e6)

--usb2dyn:ping_probe(1)
--os.exit()

local lclaw_id = 19
--local rclaw_id = 65
local LOOP_RATE_HZ = 60
local LOOP_SEC = 1/60

local t0 = unix.time()
-- Loop forever
while true do
  local t = unix.time()
  -- Sync write to both hands every frame
  local lclaw = Body.get_lgrip_command_position(1)
  local rclaw = Body.get_rgrip_command_position(1)
  -- Wait a millisecond for the motor to be ready for the read command
  unix.usleep(1e3)
  -- Read load/temperature/position/current
  local s,lall = lD.get_rx_everything(lclaw_id,usb2dyn)
--  local s,lall = lD.get_rx_position(lclaw_id,usb2dyn)

  --local s,rall = lD.get_rx_everything(rclaw_id,usb2dyn)
  -- Put everything into shared memory
  print('L',unpack(lall))
  --print('R',unpack(lall))
  -- Wait for the rate
  local t_loop = unix.time()
  local t_wait = 1e6*(LOOP_SEC - t_loop)
  unix.usleep(8e4)
end
