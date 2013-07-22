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
local libHokuyo = require'libHokuyo'

-- Setup the Hokuyos array
local hokuyos = {}

-- Acquire Hokuyo data
if OPERATING_SYSTEM=='darwin' then
  head_device = "/dev/cu.usbmodem1411"
  chest_device = "/dev/cu.usbmodem1421"
end

-- Initialize the Hokuyos
local head_hokuyo = libHokuyo.new_hokuyo(head_device)
local chest_hokuyo = libHokuyo.new_hokuyo(chest_device)

-- Head Hokuyo
if head_hokuyo then
  head_hokuyo.name = 'Head'
  local head_lidar_ch = simple_ipc.new_publisher('head_lidar') --head lidar
  table.insert(hokuyos,head_hokuyo)
  head_hokuyo.callback = function(data)
    local serialized = mp.pack({head_hokuyo.t_last,data})
    local ret = head_lidar_ch:send(serialized)
    --[[
    local t_diff = head_hokuyo.t_last - (last_head or unix.time())
    last_head = head_hokuyo.t_last
    print('head',ret,#serialized,1/t_diff..' Hz')
    --]]
    
  end
end

-- Chest Hokuyo
if chest_hokuyo then
  chest_hokuyo.name = 'Chest'
  table.insert(hokuyos,chest_hokuyo)
  local chest_lidar_ch = simple_ipc.new_publisher('chest_lidar') --chest lidar
  chest_hokuyo.callback = function(data)
    local serialized = mp.pack({chest_hokuyo.t_last,data})
    local ret = chest_lidar_ch:send(serialized)
    --[[
    local t_diff = chest_hokuyo.t_last - (last_chest or unix.time())
    last_chest = chest_hokuyo.t_last
    print('chest',ret,#serialized,1/t_diff..' Hz')
    --]]
  end
end

-- Ensure that we shutdown the devices properly
function shutdown()
  print'Shutting down the Hokuyos...'
  for i,h in ipairs(hokuyos) do
    h:stream_off()
    h:close()
    print('Closed Hokuyo',i)
  end
  error('Finished!')
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Begin to service
assert(#hokuyos>0,"No hokuyos detected!")
io.write('Servicing ',#hokuyos,' Hokuyos\n\n')
io.flush()

--[[
local fds = {}
local fd2hokuyo = {}
for i,h in ipairs(hokuyos) do
  fds[i] = h.fd
  fd2hokuyo[h.fd] = i
  h:stream_on()
end
while true do
  local status, fds_ready = unix.select(fds,0.050)
  print('status',status)
  for fd,ready in pairs(fds_ready) do
    print(fd,ready)
    if not ready then
      local hokuyo = hokuyos[ fd2hokuyo[fd] ]
      local data = hokuyo:get_scan()
      print(hokuyo,#data)
    end
  end
  local t_now = unix.time()
  local t_diff = t_now - (t_last or unix.time())
  t_last = t_now
  print(1/t_diff,'Hz')
end
return
--]]

local main = function()
  local main_cnt = 0
  local t0 = unix.time()
  while true do
    main_cnt = main_cnt + 1
    local t_now = unix.time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      for i,h in ipairs(hokuyos) do
        debug_str = debug_str..string.format(
        '\n\t%s Hokuyo was seen %5.3f seconds ago',h.name,t_now - h.t_last)
      end
      print(debug_str)
      t0 = t_now
      main_cnt = 0
    end
    coroutine.yield()
  end
end
libHokuyo.service( hokuyos, main )