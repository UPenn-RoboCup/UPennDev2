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
local Body = require'Body'
require'hcm'

local function write_body(grip_percentage, which_hand )
  if which_hand=='left' then
    jcm.set_commanded_hand( grip*vector.ones(3) )
  elseif which_hand=='right' then
    
  end
end

local leap_ch = simple_ipc.new_subscriber('leap')
local running_grip = nil
leap_ch.callback = function()
  local metadata, has_more = leap_ch:receive()
  local meta = mp.unpack(metadata)
  local t = unix.time()
  local t_diff = t-(last_grip_t or t)
  last_grip_t = t
  -- Timeout
  if t_diff>.5 then running_grip = nil end
  
  -- Simple filter
  local alpha = .25
  if not running_grip then running_grip = meta.sphereRadius end
  running_grip = meta.sphereRadius * alpha + running_grip * (1-alpha)
  
  local min, max = 60, 90
  -- 1 minus to reverse 
  local percent = 1-(running_grip-min)/(max-min)
  
  -- Set the grip
  print('Setting',percent)
  
  -- Set the shared memory
  Body.set_lgrip_percent( percent )
  
end
table.insert(wait_channels,leap_ch)

-- Setup the callbacks for ZMQ
local channel_polls = simple_ipc.wait_on_channels( wait_channels )
channel_polls:start()