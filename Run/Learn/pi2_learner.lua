dofile('../include.lua')

require('rpc')
require('pi2')
require('serialization')

--------------------------------------------------------------------------------
-- PI^2 learner
--------------------------------------------------------------------------------

-- initialize PI^2 parameters 
--------------------------------------------------------------------------------

local n_rollouts = 10             -- number of rollouts per update
local n_reused_rollouts = 5       -- number of reused rollouts
local n_pi2_updates = 500         -- number of pi2 updates
local noise_factor = 0.01         -- exploration noise scale factor
local noise_decay_factor = 1      -- exploration noise decay factor
local reevaluate_rollouts = true  -- reevaluate reused rollouts?

-- initialize RPC client 
--------------------------------------------------------------------------------
print('connecting to RPC server on PI2_EVALUATION channel...')

local pi2_client = rpc.new_client('PI2_EVALUATION')

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
print('improving PI2 policy...')

local learner = 
  pi2.learner.new(policy, noise_factor, n_rollouts, n_reused_rollouts)

learner:set_reevaluate_rollouts(reevaluate_rollouts)
learner:set_noise_decay(noise_decay_factor)

for i = 1, n_pi2_updates do
  local cost = learner:improve_policy()
  print(i, 'cost : ', cost)
end

print('done')
