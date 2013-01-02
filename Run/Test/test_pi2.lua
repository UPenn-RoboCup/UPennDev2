dofile('../include.lua')

require('pi2')
require('dmp')
require('unix')
require('numlua')
require('gnuplot')

-- initialize parameters 
--------------------------------------------------------------------------------
local nbasis = 10                -- number of basis functions
local dt = 0.005                 -- integrator time step
local tau = 0.5                  -- movement duration
local n_time_steps = tau/dt      -- number of time steps
local start_state = {0}          -- start position
local goal_state = {1}           -- goal position
local n_rollouts = 10            -- number of rollouts per pi2 iteration
local n_reused_rollouts = 5      -- number of reused rollouts
local n_pi2_iterations = 500     -- number of pi2 iterations

-- initialize one-dimensional dmp
--------------------------------------------------------------------------------
local primitive = dmp.new(1)
primitive:set_time_step(dt)
primitive:initialize(start_state, goal_state, tau)
primitive:learn_minimum_jerk_trajectory(nbasis)
primitive:reset()
local nonlinearity = primitive:get_transform_system(1):get_nonlinearity()

-- initialize cost functions and noise variances
--------------------------------------------------------------------------------

local noise_variances = {1/20*matrix.eye(#primitive:get_parameters(1))}

function evaluate_step_cost(t)
  local xdd = primitive:get_acceleration(1)
  if (t == 0.3) then
    return 0.5*xdd^2 + 10000000*(0.2 - primitive:get_position(1))^2
  else
    return 0.5*xdd^2
  end
end

function evaluate_terminal_cost(t)
  local x = primitive:get_position(1)
  local xd = primitive:get_velocity(1)
  return 10000*(xd^2 + 10*(goal_state[1] - x)^2)
end

-- intialize pi2 policy for dmp
--------------------------------------------------------------------------------
local policy = pi2.dmp_policy.new(primitive, n_time_steps)

-- define policy evaluation function to improve parameters 
function policy:evaluate(parameters)

  local t = 0
  local step_costs = {}
  primitive:set_parameters(parameters)

  -- integrate dmp
  primitive:reset()
  for i = 1, n_time_steps do
    t = (i - 1)*dt
    step_costs[i] = evaluate_step_cost(t)
    primitive:integrate()
  end
  primitive:reset()

  local terminal_cost = evaluate_terminal_cost(t)
  return step_costs, terminal_cost
end

-- intialize pi2 learner 
--------------------------------------------------------------------------------

local learner = 
  pi2.learner.new(policy, noise_variances, n_rollouts, n_reused_rollouts)

-- improve policy 
--------------------------------------------------------------------------------

local theta_initial = policy:get_parameters()

for i = 1, n_pi2_iterations do
  local cost = learner:improve_policy()
  print(i, 'cost : ', cost)
end

local theta_final = policy:get_parameters()

-- plot initial and final trajectories
--------------------------------------------------------------------------------

-- integrate initial dmp
local f_initial = {}
local s_initial = {}
local x_initial = {}
local t_initial = {}

primitive:set_parameters(theta_initial)
primitive:reset()

for i = 1, n_time_steps do
  primitive:integrate()
  t_initial[i] = (i - 1)*dt
  x_initial[i] = primitive:get_position(1)
  s_initial[i] = primitive:get_phase()
  f_initial[i] = nonlinearity:predict(s_initial[i])
end

-- integrate final dmp
local f_final = {}
local s_final = {}
local x_final = {}
local t_final = {}

primitive:set_parameters(theta_final)
primitive:reset()

for i = 1, n_time_steps do
  primitive:integrate()
  t_final[i] = (i - 1)*dt
  x_final[i] = primitive:get_position(1)
  s_final[i] = primitive:get_phase()
  f_final[i] = nonlinearity:predict(s_final[i])
end

-- plot trajectories
gnuplot.figure()
gnuplot.plot(
  {'x_initial(t)', t_initial, x_initial, '-'},
  {'x_final(t)', t_final, x_final, '-'}
)

-- plot nonlinearities
gnuplot.figure()
gnuplot.plot(
  {'f_initial(t)', s_initial, f_initial, '-'},
  {'f_final(s)', s_final, f_final, '-'}
)

-- plot cost curve 
gnuplot.figure()
gnuplot.plot(
  {'pi2 cost curve', learner:get_cost_curve(), '-'}
)
