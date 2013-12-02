---------------------------------
-- Gripper Manager for Team THOR
-- (c) Stephen McGill
---------------------------------

dofile'include.lua'
require'jcm'
require'unix'
local util = require'util'
local Body = require'Body'
local lD = require'libDynamixel'

-- Open the bus
local baud = 2e6
local usb2dyn = lD.new_bus('/dev/ttyUSB4',2e6)

local rclaw_id = 64
local rclaw_joint = Body.indexRGrip
local lclaw_id = 66
local lclaw_joint = Body.indexLGrip

local LOOP_RATE_HZ = 50
local LOOP_SEC = 1/LOOP_RATE_HZ

-- Use position mode to start
local is_torque_mode = false
lD.set_rx_torque_mode(rclaw_id,0,usb2dyn)
lD.set_rx_torque_enable(rclaw_id,0,usb2dyn)

local t0 = unix.time()
local t_debug = t0
-- Loop forever
while true do
  local t = unix.time()
  
  
  -- RIGHT HAND --
  
  -- In position mode
  if jcm.gripperPtr.torque_mode[3]==0 then
    -- Make sure we are in the right mode
    while is_torque_mode_rg do
      lD.set_rx_torque_mode(rclaw_id,0,usb2dyn)
      unix.usleep(1e2)
      local status = lD.get_rx_torque_mode(rclaw_id,usb2dyn)
      if status then
        local read_parser = lD.byte_to_number[ #status.parameter ]
        value = read_parser( unpack(status.parameter) )
        is_torque_mode_rg = value==1
      end
    end
    -- Open the hand with a position
    local rclaw = Body.get_rgrip_command_position(1)
    local rstep = Body.make_joint_step(Body.indexRGrip,rclaw)
    lD.set_rx_command_position(rclaw_id,rstep,usb2dyn)
  elseif jcm.gripperPtr.torque_mode[3]==1 then
    -- Make sure we are in the torque mode
    while not is_torque_mode_rg do
      lD.set_rx_torque_mode(rclaw_id,1,usb2dyn)
      unix.usleep(1e2)
      local status = lD.get_rx_torque_mode(rclaw_id,usb2dyn)
      if status then
        local read_parser = lD.byte_to_number[ #status.parameter ]
        value = read_parser( unpack(status.parameter) )
        is_torque_mode_rg = value==1
      end
    end
    -- Grab the torque from the user
    local r_tq_step = Body.get_rgrip_command_torque_step()
    -- Close the hand with a certain force (0 is no force)
    lD.set_rx_command_torque(rclaw_id,r_tq_step,usb2dyn)
  end

  -- Wait a millisecond for the motor to be ready for the read command
  unix.usleep(1e3)

  -- Read load/temperature/position/current
  local s, rall = lD.get_rx_everything(rclaw_id,usb2dyn)

  -- TODO: Put everything into shared memory
  if type(rall)=='table' then
    t_read = unix.time()
    jcm.sensorPtr.position[rclaw_joint] = 
      Body.make_joint_radian(rclaw_joint,rall.position)
    jcm.sensorPtr.velocity[rclaw_joint] = rall.speed
    jcm.sensorPtr.load[rclaw_joint] = rall.load
    jcm.sensorPtr.temperature[rclaw_joint] = rall.temperature
    -- time of Read
    jcm.treadPtr.position[rclaw_joint] = t_read
    jcm.treadPtr.velocity[rclaw_joint] = t_read
    jcm.treadPtr.load[rclaw_joint] = t_read
    jcm.treadPtr.temperature[rclaw_joint] = t_read
    --
    local t_read_diff = t_read - (t_read_last or t0)
    t_read_last = t_read
    if t-t_debug>1 then
      -- Debug printing
      print('R | Time diff:',t_read_diff,'Torque mode',is_torque_mode_rg)
      util.ptable(rall)
      print()
      t_debug=t
    end
  elseif type(rall)=='number' then
    print('Error?',rall)
  end
  
  
  
  
  -- LEFT HAND --
  
  
  
  -- In position mode
  if jcm.gripperPtr.torque_mode[1]==0 then
    -- Make sure we are in the right mode
    while is_torque_mode_lg do
      lD.set_rx_torque_mode(lclaw_id,0,usb2dyn)
      unix.usleep(1e2)
      local status = lD.get_rx_torque_mode(lclaw_id,usb2dyn)
      if status then
        local read_parser = lD.byte_to_number[ #status.parameter ]
        value = read_parser( unpack(status.parameter) )
        is_torque_mode_lg = value==1
      end
    end
    -- Open the hand with a position
    local lclaw = Body.get_rgrip_command_position(1)
    local rstep = Body.make_joint_step(Body.indexRGrip,lclaw)
    lD.set_rx_command_position(lclaw_id,rstep,usb2dyn)
  elseif jcm.gripperPtr.torque_mode[1]==1 then
    -- Make sure we are in the torque mode
    while not is_torque_mode_lg do
      lD.set_rx_torque_mode(lclaw_id,1,usb2dyn)
      unix.usleep(1e2)
      local status = lD.get_rx_torque_mode(lclaw_id,usb2dyn)
      if status then
        local read_parser = lD.byte_to_number[ #status.parameter ]
        value = read_parser( unpack(status.parameter) )
        is_torque_mode_lg = value==1
      end
    end
    -- Grab the torque from the user
    local r_tq_step = Body.get_rgrip_command_torque_step()
    -- Close the hand with a certain force (0 is no force)
    lD.set_rx_command_torque(lclaw_id,r_tq_step,usb2dyn)
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
    if t-t_debug>1 then
      -- Debug printing
      print('L | Time diff:',t_read_diff,'Torque mode',is_torque_mode_lg)
      util.ptable(lall)
      print()
      t_debug=t
    end
  elseif type(lall)=='number' then
    print('Error?',lall)
  end
  
  
  
  

  -- Wait for the rate
  local t_loop = unix.time()
  local t_wait = 1e6*math.min(math.max(LOOP_SEC-t_loop,LOOP_SEC),0)
  unix.usleep(t_wait)
end
