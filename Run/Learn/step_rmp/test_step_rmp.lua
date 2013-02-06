dofile('../../include.lua')

require('Body')
require('Proprioception')
require('trajectory')
require('Transform')
require('stepRMP')
require('gnuplot')
require('dcm')

local LOOP = false
local SLOW_MO = false
local PI2_RESULTS_FILE = 'results0.lua'
local MATLAB_SAVE_FILE = 'test.txt'
local PARAMETER_LOAD_FILE = '../../../Data/parameters_stepRMP_WebotsASH_train0.lua'
local PARAMETER_SAVE_FILE = '../../../Data/parameters_stepRMP_WebotsASH_learn0.lua'
local pi2_dimensions = {1, 2}
local velocity = {0, 0, 0}

-- initialize step controller
--------------------------------------------------------------------------------
Body.entry()
Proprioception.entry()
Body.update()
Proprioception.update()

local pi2_results = dofile(PI2_RESULTS_FILE)
step:entry()
step:load_parameters(PARAMETER_LOAD_FILE)
for i = 1, #pi2_dimensions do
  step:set_rmp_parameters(pi2_results.final_parameters[i], pi2_dimensions[i])
end
step:set_nominal_initialization(true)
step:set_velocity(velocity)
step:initialize_simulator_state(0.2)
step:save_parameters(PARAMETER_SAVE_FILE)
step:set_support_foot('l')

-- run step controller
--------------------------------------------------------------------------------
print('updating step controller...')

local t0 = Body.get_time()
local n_time_steps = 
  2*math.floor(step:get_parameter('step_duration')/Body.get_time_step() + 0.5)

local x_torso_rmp = {}
local y_torso_rmp = {}
local z_torso_rmp = {}
local x_cop_error = {}
local y_cop_error = {}
local x_cop_actual = {}
local y_cop_actual = {}
local x_cop_desired = {}
local y_cop_desired = {}
local x_torso_reference = {}
local y_torso_reference = {}
local z_torso_reference = {}
local x_l_foot = {}
local y_l_foot = {}
local z_l_foot = {}
local x_r_foot = {}
local y_r_foot = {}
local z_r_foot = {}
local t = {}

local x_l_offset =-step:get_velocity()[1]*step:get_parameter('step_duration')/2 
local y_l_offset = step:get_parameter('y_foot_offset')
local z_l_offset = 0
local x_r_offset = step:get_velocity()[1]*step:get_parameter('step_duration')/2
local y_r_offset =-step:get_parameter('y_foot_offset')
local z_r_offset = 0

while true do

  step:set_support_foot('l')
  step:start()
  for i = 1, n_time_steps/2 do
    Body.update()
    Proprioception.update()
    step:update()
    if (SLOW_MO) then
      unix.usleep(1e4)
    end

    t[i] = Body.get_time() - t0 
    x_torso_rmp[i] = step:get_rmp():get_position(1)
    y_torso_rmp[i] = step:get_rmp():get_position(2)
    z_torso_rmp[i] = step:get_rmp():get_position(2)
    x_cop_error[i] = pcm:get_cop(1) - mcm:get_desired_cop(1)
    y_cop_error[i] = pcm:get_cop(2) - mcm:get_desired_cop(2)
    x_cop_actual[i] = pcm:get_cop(1) + step:get_torso_state()[1][1]
                    + x_l_offset
    y_cop_actual[i] = pcm:get_cop(2) + step:get_torso_state()[1][2]
                    + y_l_offset
    x_cop_desired[i] = mcm:get_desired_cop(1) + step:get_torso_state()[1][1]
                     + x_l_offset
    y_cop_desired[i] = mcm:get_desired_cop(2) + step:get_torso_state()[1][2]
                     + y_l_offset
    x_torso_reference[i] = step:get_torso_reference_state()[1][1] 
                         + x_l_offset
    y_torso_reference[i] = step:get_torso_reference_state()[1][2] 
                         + y_l_offset
    z_torso_reference[i] = step:get_torso_reference_state()[1][3] 
                         + z_l_offset
    x_l_foot[i] = x_l_offset
    y_l_foot[i] = y_l_offset
    z_l_foot[i] = z_l_offset
    x_r_foot[i] = step:get_swing_foot_state()[1][1] 
                + x_l_offset
    y_r_foot[i] = step:get_swing_foot_state()[1][2] 
                + y_l_offset
    z_r_foot[i] = step:get_swing_foot_state()[1][3] 
                + z_l_offset
  end

  step:set_support_foot('r')
  step:start()
  for i = n_time_steps/2 + 1, n_time_steps do
    Body.update()
    Proprioception.update()
    step:update()
    if (SLOW_MO) then
      unix.usleep(1e4)
    end

    t[i] = Body.get_time() - t0 
    x_torso_rmp[i] = step:get_rmp():get_position(1)
    y_torso_rmp[i] = step:get_rmp():get_position(2)
    z_torso_rmp[i] = step:get_rmp():get_position(2)
    x_cop_error[i] = pcm:get_cop(1) - mcm:get_desired_cop(1)
    y_cop_error[i] = pcm:get_cop(2) - mcm:get_desired_cop(2)
    x_cop_actual[i] = pcm:get_cop(1) + step:get_torso_state()[1][1]
                    + x_r_offset
    y_cop_actual[i] = pcm:get_cop(2) + step:get_torso_state()[1][2]
                    + y_r_offset
    x_cop_desired[i] = mcm:get_desired_cop(1) + step:get_torso_state()[1][1]
                     + x_r_offset
    y_cop_desired[i] = mcm:get_desired_cop(2) + step:get_torso_state()[1][2]
                     + y_r_offset
    x_torso_reference[i] = step:get_torso_reference_state()[1][1] 
                         + x_r_offset
    y_torso_reference[i] = step:get_torso_reference_state()[1][2] 
                         + y_r_offset
    z_torso_reference[i] = step:get_torso_reference_state()[1][3] 
                         + z_r_offset
    x_l_foot[i] = step:get_swing_foot_state()[1][1] 
                + x_r_offset
    y_l_foot[i] = step:get_swing_foot_state()[1][2] 
                + y_r_offset
    z_l_foot[i] = step:get_swing_foot_state()[1][3] 
                + z_r_offset
    x_r_foot[i] = x_r_offset
    y_r_foot[i] = y_r_offset
    z_r_foot[i] = z_r_offset
  end

  if (not LOOP) then
    break
  end
end

print('done')

-- plot data
--------------------------------------------------------------------------------

gnuplot.figure()
gnuplot.plot(
  {'x_torso_rmp', t, x_torso_rmp, '-'},
  {'y_torso_rmp', t, y_torso_rmp, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'x_cop_error', t, x_cop_error, '-'},
  {'y_cop_error', t, y_cop_error, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'x_cop_actual', t, x_cop_actual, '-'},
  {'y_cop_actual', t, y_cop_actual, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'x_cop_desired', t, x_cop_desired, '-'},
  {'y_cop_desired', t, y_cop_desired, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'x_torso_reference', t, x_torso_reference, '-'},
  {'x_l_foot', t, x_l_foot, '-'},
  {'x_r_foot', t, x_r_foot, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'y_torso_reference', t, y_torso_reference, '-'},
  {'y_l_foot', t, y_l_foot, '-'},
  {'y_r_foot', t, y_r_foot, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'z_torso_reference', t, z_torso_reference, '-'},
  {'z_l_foot', t, z_l_foot, '-'},
  {'z_r_foot', t, z_r_foot, '-'}
)

-- save step parameters and exit
--------------------------------------------------------------------------------

local f = io.open(MATLAB_SAVE_FILE, 'w+') 
for i = 1, n_time_steps do
  f:write(string.format('%f ', x_torso_rmp[i]))
  f:write(string.format('%f ', y_torso_rmp[i]))
  f:write(string.format('%f ', z_torso_rmp[i]))
  f:write(string.format('%f ', x_cop_error[i]))
  f:write(string.format('%f ', y_cop_error[i]))
  f:write(string.format('%f ', x_cop_actual[i]))
  f:write(string.format('%f ', y_cop_actual[i]))
  f:write(string.format('%f ', x_cop_desired[i]))
  f:write(string.format('%f ', y_cop_desired[i]))
  f:write(string.format('%f ', x_torso_reference[i]))
  f:write(string.format('%f ', y_torso_reference[i]))
  f:write(string.format('%f ', z_torso_reference[i]))
  f:write(string.format('%f ', x_l_foot[i]))
  f:write(string.format('%f ', y_l_foot[i]))
  f:write(string.format('%f ', z_l_foot[i]))
  f:write(string.format('%f ', x_r_foot[i]))
  f:write(string.format('%f ', y_r_foot[i]))
  f:write(string.format('%f ', z_r_foot[i]))
  f:write(string.format('%f\n', t[i]))
end
f:close()

step:exit()
Proprioception.exit()
Body.exit()
