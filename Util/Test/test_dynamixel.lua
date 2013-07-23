dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel2'

----[[
local test_dynamixel = libDynamixel.new_bus()
print('Using',test_dynamixel.ttyname)
local found = test_dynamixel:ping_probe()
--found = {14,18}
for k,m in ipairs(found) do
  --[[
  local status, value = test_dynamixel:get_mx_status_return_level(m)
  if value then 
    print('Status return',m,value)
    if value~=1 then
      local status, value = test_dynamixel:set_mx_status_return_level(m,1)
    end
  end
  --]]

  --[[
  status, value = test_dynamixel:get_mx_delay(m)
  if value then 
    print('Return delay',m,value)
    if value>0 then
      local status, value = test_dynamixel:set_mx_delay(m,0)
    end
  end
  --]]

  --status, value = test_dynamixel:get_mx_firmware(m)
  --if value then print('Firmware',m,value) end

end

status, value = test_dynamixel:set_mx_torque_enable( found, 0 )
status, value = test_dynamixel:set_mx_led( found, 1 )
----[[
status, value = test_dynamixel:set_mx_command( found, 2048 )
if status then
  print('write return',status, value )
  for k,v in pairs(status) do print(k,v) end
end
--]]
local t0 = unix.time()
status, values = test_dynamixel:get_mx_position( found )
local t1 = unix.time()
print('Positions',unpack(values))
print('time',t1-t0)
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