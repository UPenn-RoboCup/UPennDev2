dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel'
local util = require'util'
local carray = require'carray'

--one_chain = libDynamixel.new_bus()

if not one_chain then
  if OPERATING_SYSTEM=='darwin' then
    right_arm = libDynamixel.new_bus('/dev/cu.usbserial-FTT3ABW9A')
    left_arm  = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9B'
    right_leg = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9C'
    left_leg  = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9D'
  else
    right_arm = libDynamixel.new_bus'/dev/ttyUSB0'
    left_arm  = libDynamixel.new_bus'/dev/ttyUSB1'
    right_leg = libDynamixel.new_bus'/dev/ttyUSB2'
    left_leg  = libDynamixel.new_bus'/dev/ttyUSB3'
--    grippers  = libDynamixel.new_bus('/dev/ttyUSB4',1000000)
  end
end

-- Choose a chain
--local test_dynamixel = left_arm
--local test_dynamixel = right_arm
local test_dynamixel = left_leg
--local test_dynamixel = right_leg
assert(test_dynamixel)
print('Using',test_dynamixel.ttyname)

local found = test_dynamixel:ping_probe(2)
print('Inspecting',table.concat(found,','))

local status = libDynamixel.get_mx_torque_mode(66,test_dynamixel)
print('status',status)
if status then
local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
print('Value:',value)
end

local status = libDynamixel.set_mx_command_position(1,2500,test_dynamixel)
print('status',status)
--local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
--print('Value:',value)




os.exit()

-- Change ID 19 to 64
local status = libDynamixel.get_rx_id(19,test_dynamixel)
print('status',status)
if status then
local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
print('Value:',value)
local status = libDynamixel.set_rx_id(19,64,test_dynamixel)
end

os.exit()

--[[
local found = test_dynamixel:ping_probe()
print('Inspecting',table.concat(found,','))
----]]
--found = {2,4,6,8,10,12,14,29,30,32,34,36,37}
--found = {29,30} --head
--found = {16,18,20,22,24,26} --left leg
--found = {22,24,26} --left ankle/knee

--[[
found = {29,30,37} --head and lidar

mx_ids  = {37}
nx_ids  = {29,30}
-- off
mx_vals = {0}
nx_vals = {0,0}
-- on
--mx_vals = {1}
--nx_vals = {250,250}
register = 'led'

-- on
--mx_vals = {1}
--nx_vals = {1,1}
register = 'torque_enable'

local inst = libDynamixel.mx_nx_bulk_write(
  register, mx_ids, mx_vals, nx_ids, nx_vals
)

print('bulk',type(inst))
print( string.byte(inst,1,-1) )


libDynamixel.mx_nx_bulk_write( register, mx_ids, mx_vals, nx_ids, nx_vals, test_dynamixel)



local mx_inst = libDynamixel.set_mx_led(37,1)
print()
print('mx',type(mx_inst))
print( string.byte(mx_inst,1,-1) )
local mx_status = libDynamixel.set_mx_led(37,1, test_dynamixel)
if mx_status then
  util.ptable(mx_status)
end


--libDynamixel.set_nx_led_green({29,30},{250,250}, test_dynamixel)
--libDynamixel.set_nx_led_green({29,30},{0,0}, test_dynamixel)
--]]

--found = {15,17,19,21,23,25} --right leg
found = {9,13} --right arm yaws

--os.exit()

--local status = libDynamixel.get_mx_max_torque(15,test_dynamixel)
--print( libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter)) )

for _,m in ipairs(found) do
  print(string.format('\nFound ID %2d',m))

  local model
  local status = libDynamixel.get_nx_model_num(m,test_dynamixel)
  if status then
    util.ptable(status)
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    model = libDynamixel.model_num_lookup[value]
    print(string.format('Model Number: %d',value),model)
  end
  
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
  
  local status = libDynamixel.get_nx_firmware(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Firmware: %d',value))
  end

  local status = libDynamixel.get_nx_mode(m,test_dynamixel)
  if status then
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Mode: %d',value))
  end

--[[
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
--]]
--[[
  local t0 = unix.time()
  cnt = 0
  while true do
    local status = libDynamixel.get_nx_data(24,test_dynamixel)
    if status then
      cnt = cnt+1
      -- For motor 26: 
      -- 1: pitch (labeled x on mini 58)
      -- 3: roll (y)
      if cnt%10==0 then
        local data = carray.short( string.char(unpack(status.parameter)) )
        print(util.color('External Data:','yellow'),data[1],data[2],data[3],data[4])
      end
    end
    status = nil
  end
  local t1 = unix.time()
  print('Rate:',cnt/(t1-t0),'Hz')
--]]

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
