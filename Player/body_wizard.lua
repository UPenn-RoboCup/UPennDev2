---------------------------------
-- Body Wizard for Team THOR
-- This just runs the Body entry/update/exit
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'

Body.entry()

while true do
  Body.update()
end

Body.exit()