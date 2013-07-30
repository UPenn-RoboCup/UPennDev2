dofile'include.lua'
local unix = require'unix'
local spacemouse = require 'spacemouse'
--sm = spacemouse.init(0x046d, 0xc62b) -- pro
local sm = spacemouse.init(0x046d, 0xc626) -- regular

local function process_button(data)
  print('Got button input!',data)
end

local function process_rotate(data)
  print('Got rotate input!')
  for k,v in pairs(data) do
    print(k,v)
  end
end

local function process_translate(data)
  print('Got translate input!')
  for k,v in pairs(data) do
    print(k,v)
  end
end

-- Update every 10ms
local update_interval = 0.010 * 1e6
while true do
  local t = unix.time()
  local evt, data = sm:get()
  if evt then print(t) end
  if evt=='button' then process_button(data) end
  if evt=='rotate' then process_rotate(data) end
  if evt=='translate' then process_translate(data) end
  unix.usleep(update_interval)
end