-----------------------------------------
-- Spacemouse Wizard
-- Broadcasts spacemouse input over ZMQ
-- (c) Stephen McGill, 2013
-----------------------------------------

dofile'include.lua'

--local is_debug = true

-- Libraries
local Config = require'Config'
local unix       = require'unix'
local getch      = require'getch'
local mp         = require'msgpack'
local spacemouse = require'spacemouse'
local util       = require'util'
local simple_ipc = require'simple_ipc'

local spacemouse_ch = simple_ipc.new_publisher('spacemouse')
local sm = spacemouse.init(0x046d, 0xc626) -- regular
--local sm = spacemouse.init(0x046d, 0xc62b) -- pro

local update_hz = 30
local update_interval_us = 1e6 / update_hz

------------
-- Start processing
local t_last = unix.time()
while true do
  
  -- Wait the update interval
  unix.usleep( update_interval_us )
  local t = unix.time()
  
  -- Grab the spacemouse data
  local evt, data = sm:get()
  
  -- Grab the keyboard character (modifier)
  local key_code = getch.nonblock()
  if key_code then
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
  end
  
  if evt then
    if type(data)=='number' then data = {btn=data} end
    data.t = t
    data.key = key_code
    local ret = spacemouse_ch:send(mp.pack(data))
  end
  
  -- Print debugging messages
  if t-t_last>1 and evt then
    t_last = t;
    os.execute'clear'
    print(t,ret)
    print(util.color(evt,'yellow'))
    if type(data)=='table' then
      util.ptable(data)
    else
      print(data)
    end
    if key_code then print('Key',key_code,key_char,key_char_lower) end
  end
  
end
