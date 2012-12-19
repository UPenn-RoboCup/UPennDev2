dofile('../include.lua')

require('Body')
require('Transform')
require('Kinematics')
require('stepMJT')
require('vector')
require('dcm')

local INIT_STANCE = false

-- initialize step parameters (coordinates relative to l_foot frame)
----------------------------------------------------------------------

Body.entry()
step:entry()
local qstance = step:get_stance()

-- update stance
if (INIT_STANCE) then
  print('init stance')
  Body.set_simulator_pose({0, 0, 100, 0})
  while Body.get_time() < 2 do
    dcm:set_joint_position(qstance, 'legs')
    Body.update()
  end
  Body.set_simulator_pose({0, 0, 0.577, 0})
  while true do
    Body.update()
  end
end

-- update step
print('update step')
step:reset()
while (true) do
  Body.update()
  step:update()
  unix.usleep(1e4)
end

step:exit()
Body.exit()
