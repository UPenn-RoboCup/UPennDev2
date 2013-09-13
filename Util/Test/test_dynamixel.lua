dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel'
local util = require'util'

--[[
local new_dynamixel = libDynamixel.new_bus()
local right_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5A')
local left_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5B')
local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5C')
--]]
local right_arm = libDynamixel.new_bus'/dev/ttyUSB0'
local left_arm_and_spine = libDynamixel.new_bus'/dev/ttyUSB1' --left arm
local right_leg = libDynamixel.new_bus'/dev/ttyUSB2'
local left_leg = libDynamixel.new_bus'/dev/ttyUSB3'

-- Choose a chain
local test_dynamixel = left_arm_and_spine
assert(test_dynamixel)
print('Using',test_dynamixel.ttyname)

--[[
local found = test_dynamixel:ping_probe()
for _,m in ipairs(found) do print(string.format('\nFound ID %2d',m)) end
--]]

--os.exit()

--local status = libDynamixel.get_mx_max_torque(15,test_dynamixel)
--print( libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter)) )


found = found or {}
print('Inspecting',table.concat(found,','))
for _,m in ipairs(found) do
  print(string.format('\nFound ID %2d',m))
  
  --[[
  local status = libDynamixel.get_nx_status_return_level(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Status return: %d',value))
    if value~=2 then
      local status = libDynamixel.set_nx_status_return_level(m,2,test_dynamixel)
      util.ptable(status)
    end
  else
    print('No status return??')
  end
  --]]

  --[[
  local status = libDynamixel.get_nx_delay(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Delay: %d',value))
    if value==250 then
      local status = libDynamixel.set_nx_delay(m,0,test_dynamixel)
    end
  end
  --]]

  --[[
  local status = libDynamixel.get_nx_model_num(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Model Number: %d',value))
  end
  --]]
  
  --[[
  local status = libDynamixel.get_nx_firmware(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Firmware: %d',value))
  end
  --]]
  
  --[[
  local status = libDynamixel.get_mx_position(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Position: %d',value))
  end
  --]]

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

--]]
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