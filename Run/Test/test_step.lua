dofile('../include.lua')

require('Body')
require('Transform')
require('Kinematics')
require('stepMJT')
require('vector')
require('dcm')

--local INIT_STANCE = true
local INIT_STANCE = false

-- initialize step parameters (coordinates relative to l_foot frame)
----------------------------------------------------------------------

Body.entry()
step:entry()

local q_start = step:get_joint_start_position()
local v_torso = step:get_torso_start_velocity()

-- update stance
if (INIT_STANCE) then
  print('init stance')
  Body.set_simulator_torso_frame(Transform.pose6D{0, 0, 100})
  while Body.get_time() < 2 do
    dcm:set_joint_position(q_start, 'legs')
    Body.update()
  end
  Body.set_simulator_torso_frame(Transform.pose6D{0, 0, 0.577})
  while true do
    Body.update()
  end
end

-- update step
print('update step')
step:reset()
Body.set_simulator_torso_twist({v_torso[1], v_torso[2], v_torso[3], 0, 0, 0})
while (true) do
  Body.update()
  step:update()
  unix.usleep(1e4)
end

step:exit()
Body.exit()
