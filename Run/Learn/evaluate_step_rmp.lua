dofile('../include.lua')

require('pi2')
require('rpc')
require('mcm')
require('pcm')
require('stepRMP')
require('Proprioception')
require('Body')
require('unix')

--------------------------------------------------------------------------------
-- Evaluate stepRMP
--------------------------------------------------------------------------------

UPDATE_INITIAL_STATE = true
RESET_SIMULATOR = false

-- initialize parameters
--------------------------------------------------------------------------------

local velocity = {0.2, 0, 0}
local dimensions = {1, 2}  -- active learning dimensions
local parameter_file = '../../Data/parameters_stepRMP_WebotsASH_eval.lua'

local function compute_cost_to_go(step_costs, terminal_cost)
   local cost = terminal_cost
   for i = 1,#step_costs do
     cost = cost + step_costs[i]
   end
  return cost
end

-- intialize step controller
--------------------------------------------------------------------------------

Body.entry()
Proprioception.entry()
Body.update()
Proprioception.update()

step:entry()
step:load_parameters(parameter_file)
step:set_nominal_initialization(true)
step:set_velocity(velocity)
step:set_support_foot('l')
step:initialize()

local n_time_steps = step:get_parameter('step_duration')/Body.get_time_step()
n_time_steps = math.floor(n_time_steps + 0.5)

-- define cost functions
--------------------------------------------------------------------------------

local function evaluate_step_cost()
  local cost = 0
  -- penalize squared acceleration
  for i = 1, #dimensions do
    cost = cost + 0.5*step:get_rmp():get_acceleration(dimensions[i])^2
  end
  -- penalize square CoP error
  local cop_error = 0
  for i = 1, 2 do
    cop_error = cop_error + (pcm:get_cop(i) - mcm:get_desired_cop(i))^2
  end
  cost = cost + 1e10*math.sqrt(cop_error)
  -- penalize tipping
  if (mcm:get_tipping_status(1) == 1) then
    cost = cost + 1e12
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
  Body.set_simulator_torso_twist(torso_twist)

  step:start()
  for i = 1, self.n_time_steps do
    Body.update()
    Proprioception.update()
    step:update()
    step_costs[i] = evaluate_step_cost()
  end

  local terminal_cost = evaluate_terminal_cost(t)

  local cost_to_go = compute_cost_to_go(step_costs, terminal_cost)
  print('cost', cost_to_go)

  if (UPDATE_INITIAL_STATE and noiseless) then
    local rmp_state = step:get_rmp():get_state()
    for i = 1, 3 do
      rmp_state[2][i] = -rmp_state[2][i] -- y is antiperiodic
    end
    step:set_parameter('nominal_rmp_state', rmp_state)
  end

  return step_costs, terminal_cost
end

-- start rpc server
--------------------------------------------------------------------------------

local pi2_server = rpc.new_server('PI2_EVALUATION')
pi2_server:set_timeout(nil)

function ping()
end

while (not RESET_SIMULATOR) do
  pi2_server:update()
end

step:save_parameters(parameter_file)
step:exit()
Proprioception.exit()
Body.exit()
Body.reset_simulator()
