dofile('../include.lua')

require('rpc')
require('pi2')
require('gnuplot')
require('serialization')

--------------------------------------------------------------------------------
-- PI^2 learner
--------------------------------------------------------------------------------

local OUTPUT_FILE = arg[1] or 'Results/pi2_results.lua'
local rpc_endpoint = 'tcp://localhost:12012'
local context = zmq.init()

-- initialize PI^2 parameters 
--------------------------------------------------------------------------------

local n_rollouts = 15             -- number of rollouts per update
local n_reused_rollouts = 5       -- number of reused rollouts
local n_pi2_updates = 100         -- number of pi2 updates
local noise_factor = 0.001        -- exploration noise scale factor
local noise_decay_factor = 0.97   -- exploration noise decay factor
local reevaluate_rollouts = false -- reevaluate reused rollouts?

-- initialize RPC client 
--------------------------------------------------------------------------------
print('connecting to RPC server on PI2_EVALUATION channel...')

local pi2_client = rpc.client.new(rpc_endpoint, context)

pi2_client:set_timeout(0.1)
while (not pi2_client:connect()) do
end

print('done')

-- initialize remote PI^2 policy
--------------------------------------------------------------------------------
print('initializing PI2 policy...')

pi2_client:set_timeout(1)
local success, n_dimensions = pi2_client:call('policy:get_n_dimensions')
assert(success)
local success, n_time_steps = pi2_client:call('policy:get_n_time_steps')
assert(success)
local success, noise_variances = pi2_client:call('policy:get_noise_variances')
assert(success)
local success, initial_parameters = pi2_client:call('policy:get_parameters')
assert(success)
local success, basis_vectors = pi2_client:call('policy:get_basis_vectors')
assert(success, basis_vectors)
local success, temporal_weights = pi2_client:call('policy:get_temporal_weights')
assert(success)

local policy = pi2.policy.new(n_dimensions, n_time_steps)

function policy:get_noise_variances()
  return noise_variances
end

function policy:get_parameters()
  return initial_parameters
end

function policy:get_basis_vectors()
  return basis_vectors
end

function policy:get_temporal_weights()
  return temporal_weights
end

function policy:evaluate(parameters, noiseless)
  pi2_client:set_timeout(0.1)
  while (not pi2_client:call('ping')) do
  end
  pi2_client:set_timeout(60)
  local success, step_costs, terminal_cost =
    pi2_client:call('policy:evaluate', parameters, noiseless)
  assert(success, step_costs)
  return step_costs, terminal_cost
end

print('done')

-- initiliaze PI^2 learner
--------------------------------------------------------------------------------

print('initializing PI2 policy...')

local learner = 
  pi2.learner.new(policy, noise_factor, n_rollouts, n_reused_rollouts)

learner:set_reevaluate_rollouts(reevaluate_rollouts)
learner:set_noise_decay(noise_decay_factor)

local function save_pi2_results(filename)
  local serialize = serialization.serialize
  local costs = learner:get_costs()
  local rollout_costs = learner:get_rollout_costs()
  local best_rollout = learner:get_rollouts()[1]

  local best_parameters = {}
  local final_parameters = {}
  for i = 1, #initial_parameters do
    best_parameters[i] = best_rollout.parameters[i]:totable()
    final_parameters[i] = learner.parameters[i]:totable()
  end

  local f = io.open(filename, 'w+') 
  f:write('return {')
  f:write('costs = '..serialize(costs)..',')
  f:write('rollout_costs = '..serialize(rollout_costs)..',')
  f:write('initial_parameters = '..serialize(initial_parameters)..',')
  f:write('final_parameters = '..serialize(final_parameters)..',')
  f:write('best_parameters = '..serialize(best_parameters)..',')
  f:write('}')
  f:close()
end

print('done')

-- run experiment
--------------------------------------------------------------------------------

print('improving PI2 policy...')

gnuplot.figure()
for i = 1, n_pi2_updates do
  local cost = learner:improve_policy()
  local rollouts = learner:get_rollouts()
  for k = 1, #rollouts do
    print(i, '------------> ', rollouts[k].cost)
  end
  print(i, 'update cost : ', cost)
  gnuplot.plot('update cost', learner:get_costs(), '-')
  save_pi2_results(OUTPUT_FILE)
  save_pi2_results(OUTPUT_FILE..'.backup')
end

print('done')
