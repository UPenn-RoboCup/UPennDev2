dofile('../../include.lua')

require('pi2')
require('rpc')
require('mcm')
require('pcm')
require('stepRMP')
require('Proprioception')
require('Platform')
require('unix')

--------------------------------------------------------------------------------
-- Evaluate stepRMP
--------------------------------------------------------------------------------

RESET_SIMULATOR = false

-- initialize parameters
--------------------------------------------------------------------------------

local velocity = {0, 0, 0}
local dimensions = {1, 2}  -- active learning dimensions
local parameter_load_file = '../../../Data/parameters_stepRMP_WebotsASH_train0.lua'
local parameter_save_file = '../../../Data/parameters_stepRMP_WebotsASH_eval0.lua'

local function compute_cost_to_go(step_costs, terminal_cost)
   local cost = terminal_cost
   for i = 1,#step_costs do
     cost = cost + step_costs[i]
   end
  return cost
end

-- initialize step controller
--------------------------------------------------------------------------------

Platform.entry()
Proprioception.entry()
Platform.update()
Proprioception.update()

step:entry()
step:load_parameters(parameter_load_file)
step:set_nominal_initialization(true)
step:set_velocity(velocity)
step:set_support_foot('l')
step:initialize()

local n_time_steps = 2*step:get_parameter('step_duration')/Platform.get_time_step()
n_time_steps = math.floor(n_time_steps + 0.5)

-- define cost functions
--------------------------------------------------------------------------------

local function evaluate_step_cost()

  -- get squared acceleration cost
  local accel_cost = 0
  for i = 1, #dimensions do
    accel_cost = accel_cost + step:get_rmp():get_acceleration(dimensions[i])^2
  end
  accel_cost = math.sqrt(accel_cost)

  -- get squared CoP error cost
  local cop_cost = 0
  for i = 1, 2 do
    cop_cost = cop_cost + (pcm:get_cop(i) - mcm:get_desired_cop(i))^2
  end
  cop_cost = math.sqrt(cop_cost)

  -- get tipping cost
  local tipping_cost = 0
  if (mcm:get_tipping_status(1) == 1) then
    tipping_cost = 1
  end

  return 0*accel_cost + 1e6*(cop_cost + 10*tipping_cost)
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

  -- initialize simulator state 
  step:initialize_simulator_state(0.2)

  -- run trial to get step costs
  step:set_support_foot('l')
  step:start()
  for i = 1, self.n_time_steps/2 do
    Platform.update()
    Proprioception.update()
    step:update()
    step_costs[i] = evaluate_step_cost()
  end

  step:set_support_foot('r')
  step:start()
  for i = self.n_time_steps/2 + 1, self.n_time_steps do
    Platform.update()
    Proprioception.update()
    step:update()
    step_costs[i] = evaluate_step_cost()
  end

  local terminal_cost = evaluate_terminal_cost(t)

  local cost_to_go = compute_cost_to_go(step_costs, terminal_cost)
  print('cost', string.format('%g', cost_to_go))
  return step_costs, terminal_cost
end

-- start rpc server
--------------------------------------------------------------------------------

local pi2_server = rpc.server.new('PI2_EVALUATION')
pi2_server:set_timeout(nil)

function ping()
end

while (not RESET_SIMULATOR) do
  pi2_server:update()
end

step:save_parameters(parameter_save_file)

step:exit()
Proprioception.exit()
Platform.exit()
Platform.reset_simulator()
