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
local getch = require'getch'
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
  
  local g_min, g_max = 57, 90
  local clamped = math.min( math.max(running_grip,g_min),g_max )
  local percent = (clamped-g_min) / (g_max-g_min)
  
  -- Set the grip
  print('Setting',clamped,percent)
  
  -- Set the shared memory
  if leap_enabled and leap_dir=='left' then
    Body.set_lgrip_percent( percent )
  end
  if leap_enabled and leap_dir=='right' then
    Body.set_rgrip_percent( percent )
  end
  
  -- Grab the keyboard character
  local key_code = getch.nonblock()
  if key_code then
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
    print('Key',key_code,key_char,key_char_lower)
    if key_char_lower=='e' then
      leap_enabled = true
    elseif key_char_lower=='d' then
      leap_enabled = false
    elseif key_char_lower=='r' then
      leap_dir = 'right'
    elseif key_char_lower=='l' then
      leap_dir = 'left'
    end
  end
  
end
table.insert(wait_channels,leap_ch)

-- Setup the callbacks for ZMQ
local channel_polls = simple_ipc.wait_on_channels( wait_channels )
channel_polls:start()