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

-- plot trajectories
--------------------------------------------------------------------------------
local torso_trajectory = step:get_torso_trajectory() 
local foot_trajectory = step:get_swing_foot_trajectory() 
local cop_trajectory = step:get_cop_trajectory() 

local torso_position = {{}, {}, {}}
local foot_position = {{}, {}, {}}
local desired_cop = {{}, {}, {}}
local t = {}

for i = 0, 5000 do
  t[i] = i*parameters.step_duration/5000
  for j = 1, 3 do
    torso_position[j][i] = torso_trajectory[j](t[i])
    foot_position[j][i] = foot_trajectory[j](t[i])
    desired_cop[j][i] = cop_trajectory[j](t[i])
  end
end

gnuplot.figure()
gnuplot.plot(
  {'torso_x', t, torso_position[1], '-'},
  {'torso_y', t, torso_position[2], '-'},
  {'torso_z', t, torso_position[3], '-'},
  {'desired_cop_x', t, desired_cop[1], '-'},
  {'desired_cop_y', t, desired_cop[2], '-'},
)

gnuplot.figure()
gnuplot.plot(
  {'swing_foot_x', t, foot_position[1], '-'},
  {'swing_foot_y', t, foot_position[2], '-'},
  {'swing_foot_z', t, foot_position[3], '-'},
  {'desired_cop_x', t, desired_cop[1], '-'},
  {'desired_cop_y', t, desired_cop[2], '-'},
)

-- exit
--------------------------------------------------------------------------------
step:exit()
Proprioception.exit()
Body.exit()

