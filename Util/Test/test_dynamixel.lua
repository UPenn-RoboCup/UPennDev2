dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel'

----[[
--local test_dynamixel = libDynamixel.new_bus()
local test_dynamixel = libDynamixel.new_bus('/dev/tty.usbserial-FTT3AAV5B')
assert(test_dynamixel)

print('Using',test_dynamixel.ttyname)

local found = test_dynamixel:ping_probe()
for _,m in ipairs(found) do print(string.format('\nFound ID %2d',m)) end

os.exit()

local status = libDynamixel.get_mx_max_torque(15,test_dynamixel)
print( libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter)) )



--found = {14,18}
for _,m in ipairs(found) do
  print(string.format('\nFound ID %2d',m))
  local status = libDynamixel.get_nx_status_return_level(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Status return: %d',value))
    if value==2 then
      local status = libDynamixel.set_nx_status_return_level(m,1,test_dynamixel)
    end
  end

  local status = libDynamixel.get_nx_delay(m,test_dynamixel)
  if status then 
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Delay: %d',value))
    if value==250 then
      local status = libDynamixel.set_nx_delay(m,0,test_dynamixel)
    end
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

end


status = libDynamixel.set_nx_torque_enable( 12, 1, test_dynamixel )
status = libDynamixel.set_nx_led_green( 12, 255 ,test_dynamixel )
status = libDynamixel.set_nx_command_position( 12, 0, test_dynamixel )
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
if true then return end
--]]