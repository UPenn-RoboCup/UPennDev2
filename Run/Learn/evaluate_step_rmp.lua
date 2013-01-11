dofile('../include.lua')

require('pi2')
require('rpc')
require('mcm')
require('pcm')
require('stepRMP')
require('Proprioception')
require('Body')

--------------------------------------------------------------------------------
-- Evaluate stepRMP
--------------------------------------------------------------------------------

RESET_SIMULATOR = false

-- initialize parameters
--------------------------------------------------------------------------------

local velocity = {0, 0, 0}
local dimensions = {1}  -- active learning dimensions
local parameter_file = '../../Data/parameters_stepRMP_WebotsASH_eval.lua'

-- intialize step controller
--------------------------------------------------------------------------------

Body.entry()
Proprioception.entry()
Body.update()
Proprioception.update()

step:entry()
step:set_support_foot('l')
step:set_velocity(velocity)
step:load_parameters(parameter_file)
step:set_nominal_initialization(true)
step:initialize()

local n_time_steps = step:get_parameter('step_duration')/Body.get_time_step()

-- define cost functions
--------------------------------------------------------------------------------

local function evaluate_step_cost()
  local cost = 0
  -- calculate acceleration cost
  for i = 1, #dimensions do
    cost = cost + 0.5*step:get_rmp():get_acceleration(dimensions[i])^2
  end
  -- calculate CoP error cost
  for i = 1, 2 do
    cost = cost + (pcm:get_cop(i) - mcm:get_desired_cop(i))^2
  end
  return cost 
end

local function evaluate_terminal_cost()
  return 0
end

-- define pi2 policy
--------------------------------------------------------------------------------

policy = pi2.rmp_policy.new(step:get_rmp(), n_time_steps, dimensions)

function policy:evaluate(parameters, noiseless)
  RESET_SIMULATOR = true

  local step_costs = {}
  for i = 1, self.n_dimensions do
    step:set_rmp_parameters(parameters[i], self.dimensions[i])
  end

  local torso_twist = step:get_torso_state()[2]
  torso_twist[4] = 0
  torso_twist[5] = 0
  torso_twist[6] = 0
  Body.set_simulator_torso_twist(torso_twist)

  step:start()
  for i = 1, self.n_time_steps do
    Body.update()
    Proprioception.update()
    step:update()
    step_costs[i] = evaluate_step_cost()
  end

  local terminal_cost = evaluate_terminal_cost(t)

  if (noiseless) then
    step:set_nominal_rmp_state(step:get_rmp():get_state())
  end
  return step_costs, terminal_cost
end

-- start rpc server
--------------------------------------------------------------------------------

local server = rpc.new_server('PI2_EVALUATION')
server:set_timeout(nil)

while (not RESET_SIMULATOR) do
  server:update()
end

step:save_parameters(parameter_file)
step:exit()
Proprioception.exit()
Body.exit()
Body.reset_simulator()
