-----------------------------------------------------------------
-- LEAP Motion listener
-- Listens to leap messages from ZeroMQ and puts to shared memory
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

-- Libraries
local unix = require'unix'
local simple_ipc = require'simple_ipc'
local mp = require'msgpack'
local wait_channels = {}

-- Memory
require'jcm'
require'hcm'

local leap_ch = simple_ipc.new_subscriber('leap')
leap_ch.callback = function()
  local metadata, has_more = leap_ch:receive()
  local meta = mp.unpack(metadata)
  --for k,v in pairs(meta) do print(k,v) end
  hcm.set_leap_sphere(meta.sphereRadius)
  hcm.set_leap_timestamp(meta.timestamp)
  
  local min, max = 60, 90
  
  --if meta.sphereRadius>max or meta.sphereRadius<min then return end
  local grip = math.min( math.max(meta.sphereRadius,min),max )
  
  -- Percent grip
  grip = 100*(meta.sphereRadius-min)/(max-min)
  
  -- Check the time diff
  local t_diff = unix.time()-(last_grip_t or unix.time())
  if t_diff>1/2 then last_grip = grip end

  -- Disallow big changes
  last_grip = last_grip or grip
  local grip_diff = grip - (last_grip or grip)
  if math.abs(grip_diff)>40 then return end
  
  -- Filter
  local alpha = .5
  grip = alpha*grip + (1-alpha)*last_grip
  
  -- Set the grip
  print('Setting',grip)
  
  -- Set the last grip
  last_grip = grip
  last_grip_t = unix.time()
  
  -- Set the shared memory
  jcm.set_commanded_hand( grip*vector.ones(3) )
  
end
table.insert(wait_channels,leap_ch)

-- Setup the callbacks for ZMQ
local channel_polls = simple_ipc.wait_on_channels( wait_channels )
channel_polls:start()