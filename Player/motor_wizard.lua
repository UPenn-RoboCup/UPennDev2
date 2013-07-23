-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'include.lua'

-- Libraries
local unix = require 'unix'
local signal = require'signal'
local carray = require'carray'
local mp = require 'msgpack'
local simple_ipc = require'simple_ipc'
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

--status, value = test_dynamixel:set_mx_torque_enable( found, 0 )
status, value = test_dynamixel:set_mx_led( found, 1 )
status, value = test_dynamixel:set_mx_command( found, 2048 )
if status then
  print('write return',status, value )
  for k,v in pairs(status) do print(k,v) end
end
if true then return end
--]]

-- Setup the dynamixels array
local dynamixels = {}

-- Initialize the dynamixels
local spine_dynamixel = libDynamixel.new_bus(chest_device)

-- Spine dynamixel
if spine_dynamixel then
  spine_dynamixel.name = 'Spine'
  table.insert(dynamixels,spine_dynamixel)
  local last_spine = unix.time()
  local spine_cnt = 0
  spine_dynamixel.callback = function(data)
    spine_cnt = spine_cnt+1
    ----[[
    local t_diff = spine_dynamixel.t_last - last_spine
    if t_diff>1 then
      local spine_fps = spine_cnt / t_diff
      local spine_debug = string.format('%s chain running at %.2f Hz | Position: %d.',
      spine_dynamixel.name,spine_fps,data)
      print(spine_debug)
      spine_cnt = 0
      last_spine = spine_dynamixel.t_last
    end
    --]]
  end
end

-- Begin to service
assert(#dynamixels>0,"No dynamixels detected!")
io.write('Servicing ',#dynamixels,' dynamixels\n\n')
io.flush()



local main = function()
  local main_cnt = 0
  local t0 = unix.time()
  
  while true do
    main_cnt = main_cnt + 1
    local t_now = unix.time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      for i,d in ipairs(dynamixels) do
        debug_str = debug_str..string.format(
        '\n\tDynamixel %s chain was seen %5.3f seconds ago',
        d.name,t_now-d.t_last)
      end
      print(debug_str)
      t0 = t_now
      main_cnt = 0
    end
    coroutine.yield()
  end
end

libDynamixel.service( dynamixels, main )