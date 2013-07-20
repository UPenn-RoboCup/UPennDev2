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

-- Publish Hokuyo data
local head_lidar_ch = simple_ipc.new_publisher('head_lidar') --head lidar
local chest_lidar_ch = simple_ipc.new_publisher('chest_lidar') --chest lidar

-- Acquire Hokuyo data
if OPERATING_SYSTEM=='darwin' then
  head_device = "/dev/cu.usbmodem1411"
  chest_device = "/dev/cu.usbmodem1421"
end
local hokuyos = {}
local head_hokuyo = libHokuyo.new_hokuyo(head_device)
if head_hokuyo then
  table.insert(hokuyos,head_hokuyo)
end
local chest_hokuyo = libHokuyo.new_hokuyo(chest_device)
if chest_hokuyo then
  table.insert(hokuyos,chest_hokuyo)
end

-- Ensure that we shutdown the devices properly
function shutdown()
  print'Shutting down the Hokuyos...'
  if head_hokuyo then
    head_hokuyo:stream_off()
    head_hokuyo:close()
    print'Closed Head Hokuyo'
  end
  if chest_hokuyo then
    chest_hokuyo:stream_off()
    chest_hokuyo:close()
    print'Closed Chest Hokuyo'
  end
  error()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Begin to service
assert(#hokuyos>0,"No hokuyos detected!")
libHokuyo.service( hokuyos )
