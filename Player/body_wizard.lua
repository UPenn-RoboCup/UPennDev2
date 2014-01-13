---------------------------------
-- Body Wizard for Team THOR
-- This just runs the Body entry/update/exit
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
require'unix'

Body.entry()

while true do
  Body.update()
  unix.usleep(Body.update_cycle*1e3)
end

Body.exit()