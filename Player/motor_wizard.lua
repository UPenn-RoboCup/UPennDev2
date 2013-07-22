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
local libDynamixel = require'libdynamixel'

-- Setup the dynamixels array
local dynamixels = {}

-- Acquire dynamixel data
if OPERATING_SYSTEM=='darwin' then
  head_device = "/dev/cu.usbmodem1411"
  chest_device = "/dev/cu.usbmodem1421"
end

-- Initialize the dynamixels
local head_dynamixel = libDynamixel.new_dynamixel_bus(head_device)
local chest_dynamixel = libDynamixel.new_dynamixel_bus(chest_device)

-- Head dynamixel
if head_dynamixel then
  head_dynamixel.name = 'Head'
  local head_lidar_ch = simple_ipc.new_publisher('head_lidar') --head lidar
  table.insert(dynamixels,head_dynamixel)
  head_dynamixel.callback = function(data)
    local serialized = mp.pack({head_dynamixel.t_last,data})
    local ret = head_lidar_ch:send(serialized)
    --[[
    local t_diff = head_dynamixel.t_last - (last_head or unix.time())
    last_head = head_dynamixel.t_last
    print('head',ret,#serialized,1/t_diff..' Hz')
    --]]
    
  end
end

-- Chest dynamixel
if chest_dynamixel then
  chest_dynamixel.name = 'Chest'
  table.insert(dynamixels,chest_dynamixel)
  local chest_lidar_ch = simple_ipc.new_publisher('chest_lidar') --chest lidar
  chest_dynamixel.callback = function(data)
    local serialized = mp.pack({chest_dynamixel.t_last,data})
    local ret = chest_lidar_ch:send(serialized)
    --[[
    local t_diff = chest_dynamixel.t_last - (last_chest or unix.time())
    last_chest = chest_dynamixel.t_last
    print('chest',ret,#serialized,1/t_diff..' Hz')
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
      io.write('\nMain loop: ',math.floor(main_cnt/t_diff),' Hz\n')
      for i,h in ipairs(dynamixels) do
        io.write(h.name,' dynamixel is alive\n')
      end
      io.flush()
      t0 = t_now
      main_cnt = 0
    end
    coroutine.yield()
  end
end
libdynamixel.service( dynamixels, main )