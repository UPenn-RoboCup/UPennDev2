dofile('../include.lua')

require('Body')
require('Proprioception')
require('Transform')
require('Kinematics')
require('stepMJT')
require('vector')
require('gnuplot')
require('dcm')

local INIT_STANCE = false
local PARAMETER_MODULE = 'parameters_stepMJT_WebotsASH_f02'
local SLOW_MO = false

-- load parameters
--------------------------------------------------------------------------------

local success, parameters = pcall(require, PARAMETER_MODULE)
if (not success) then
  print('Could not load parameter module', PARAMETER_MODULE)
  return
end

-- init controller
--------------------------------------------------------------------------------
Body.entry()
Proprioception.entry()
step:set_parameters(parameters)
step:entry()

local q_start = step:get_joint_start_position()
local v_torso = step:get_torso_start_velocity()

-- update stance
--------------------------------------------------------------------------------
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
--------------------------------------------------------------------------------
print('update step')
Body.set_simulator_torso_twist({v_torso[1], v_torso[2], v_torso[3], 0, 0, 0})
step:reset()
while (step:is_active()) do
  Body.update()
  Proprioception.update()
  step:update()
  if (SLOW_MO) then
    unix.usleep(1e4)
  end
end

-- exit
--------------------------------------------------------------------------------
step:exit()
Proprioception.exit()
Body.exit()
