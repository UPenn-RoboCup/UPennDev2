dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel'
local util = require'util'
local carray = require'carray'

--local new_dynamixel = libDynamixel.new_bus()

local right_arm = libDynamixel.new_bus'/dev/ttyUSB0'
local left_arm  = libDynamixel.new_bus'/dev/ttyUSB1'
local right_leg = libDynamixel.new_bus'/dev/ttyUSB2'
local left_leg  = libDynamixel.new_bus'/dev/ttyUSB3'

if OPERATING_SYSTEM=='darwin' then
  right_arm = libDynamixel.new_bus('/dev/cu.usbserial-FTT3ABW9A')
  left_arm  = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9B'
  right_leg = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9C'
  left_leg  = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9D'
end

-- Choose a chain
local test_dynamixel = right_leg
assert(test_dynamixel)
print('Using',test_dynamixel.ttyname)

--[[
local found = test_dynamixel:ping_probe()
print('Inspecting',table.concat(found,','))
----]]
--found = {2,4,6,8,10,12,14,29,30,32,34,36,37}
--found = {29,30} --head
found = {16,18,20,22,24,26} --left leg
--found = {--[[24,]]26} --left ankle

found = {15,17,19,21,23,25} --right leg

--os.exit()

--local status = libDynamixel.get_mx_max_torque(15,test_dynamixel)
--print( libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter)) )

for _,m in ipairs(found) do
  print(string.format('\nFound ID %2d',m))
  
  
  local status = libDynamixel.get_nx_status_return_level(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Status return: %d',value))
    --[[
    if value~=2 then
      local status = libDynamixel.set_nx_status_return_level(m,2,test_dynamixel)
      util.ptable(status)
    end
    --]]
  end
  
  local status = libDynamixel.get_nx_delay(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Delay: %d',value))
    --[[
    if value==250 then
      local status = libDynamixel.set_nx_delay(m,0,test_dynamixel)
    end
    --]]
  end

  local status = libDynamixel.get_nx_model_num(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Model Number: %d',value))
  end
  
  local status = libDynamixel.get_nx_firmware(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Firmware: %d',value))
  end

  local status = libDynamixel.get_nx_homing_offset(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Homing Offset: %d',value))
  end

  local status = libDynamixel.get_nx_position_p(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('P: %d',value))
  end
  local status = libDynamixel.get_nx_position_i(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('I: %d',value))
  end
  local status = libDynamixel.get_nx_position_d(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('D: %d',value))
  end

  local status = libDynamixel.get_nx_command_velocity(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Command Vel: %d',value))
  end

  local status = libDynamixel.get_nx_command_acceleration(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Command Acc: %d',value))
  end

  local status = libDynamixel.get_nx_data(m,test_dynamixel)
  if status then
    local data = carray.short( string.char(unpack(status.parameter)) )
    -- For motor 26: 
    -- 1: pitch (labeled x on mini 58)
    -- 3: roll (y)

    print(util.color('External Data:','yellow'),data[1],data[2],data[3],data[4])
  end
  
  --[[
  local status = libDynamixel.get_mx_position(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Position: %d',value))
  end
  --]]

end

-- SET the PID values
--local status = libDynamixel.set_nx_position_p(found,64,test_dynamixel)

os.exit()

-- Poll ext data
while true do

  unix.usleep(2e5)
  os.execute'clear'

  -- TOUCH SENSORS
  local status = libDynamixel.get_nx_data(22,test_dynamixel)
  local knee_data = carray.short( string.char(unpack(status.parameter)) )
  print(util.color('Knee Data:','yellow'),
    knee_data[1],knee_data[2],knee_data[3],knee_data[4])

  local status = libDynamixel.get_nx_data(24,test_dynamixel)
  local ank_pitch_data = carray.short( string.char(unpack(status.parameter)) )
  print(util.color('Pitch Data:','yellow'),
    ank_pitch_data[1],ank_pitch_data[2],ank_pitch_data[3],ank_pitch_data[4])

  local status = libDynamixel.get_nx_data(26,test_dynamixel)
  local ank_roll_data = carray.short( string.char(unpack(status.parameter)) )
  print(util.color('Roll Data:','yellow'),
    ank_roll_data[1],ank_roll_data[2],ank_roll_data[3],ank_roll_data[4])

end

--[[
status = libDynamixel.set_nx_torque_enable( 12, 1, test_dynamixel )
status = libDynamixel.set_nx_led_green( 12, 255 ,test_dynamixel )
status = libDynamixel.set_nx_command_position( 12, 0, test_dynamixel )
--]]
--[[
if status then
  print('write return',status, value )
  for k,v in pairs(status) do print(k,v) end
end
--]]
--[[
local t0 = unix.time()
status, values = test_dynamixel:get_mx_position( found )
local t1 = unix.time()
print('Positions',unpack(values))
print('time',t1-t0)
--]]
--[[
if status then
  print("read status",status, unpack(values) )
  for k,v in pairs(status) do
    print(k,v)
    for kk,vv in pairs(v) do
      print(kk,vv)
    end
  end
end
--]]


--[[
found = {2,4,6,8,10,12,14,30}--,32,34,36,37
while true do
os.execute('clear')
print('Positions')
local pos_tbl = {}
for _,m in ipairs(found) do
  local status = libDynamixel.get_nx_position(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    table.insert( pos_tbl,string.format('ID %d: %d',m,value) )
  end
end
print(table.concat(pos_tbl,'\n'))
unix.usleep(1e5)
end
--]]