---------------------------------
-- Body Wizard for Team THOR
-- This just runs the Body entry/update/exit
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
require'unix'

Body.entry()

local t_wait = Body.update_cycle
local t_wait_us = t_wait*1e6
local t_last = Body.get_time()

while true do
  local t = Body.get_time()
  local t_diff = t-t_last

  if t_diff>t_wait then
    --print('Update',t)
    Body.update()
    -- Webots must always update...
    if not IS_WEBOTS then unix.usleep(t_wait_us) end
  else
    if not IS_WEBOTS then print('NOP CYCLE') end
    Body.nop()
  end

end

Body.exit()
