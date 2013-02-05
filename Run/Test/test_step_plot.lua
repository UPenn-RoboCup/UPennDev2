dofile('../include.lua')

require('stepMJT')
require('gnuplot')

local PARAMETER_MODULE = 'parameters_stepMJT_WebotsASH_0'

-- load parameters
--------------------------------------------------------------------------------

local success, parameters = pcall(require, PARAMETER_MODULE)
if (not success) then
  print('Could not load parameter module', PARAMETER_MODULE)
  return
end

-- init controller
--------------------------------------------------------------------------------
step:set_parameters(parameters)
step:entry()

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
gnuplot.grid(true)
gnuplot.plot(
  {'torso_x', t, torso_position[1], '-'},
  {'torso_y', t, torso_position[2], '-'}
)

gnuplot.figure()
gnuplot.grid(true)
gnuplot.plot(
  {'swing_foot_x', t, foot_position[1], '-'},
  {'swing_foot_y', t, foot_position[2], '-'},
  {'swing_foot_z', t, foot_position[3], '-'},
  {'desired_cop_x', t, desired_cop[1], '-'},
  {'desired_cop_y', t, desired_cop[2], '-'}
)
