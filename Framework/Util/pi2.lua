require('numlua')

--------------------------------------------------------------------------------
-- pi2 : Path Integrals for Policy Improvement
--------------------------------------------------------------------------------

--[[

  author: Mike Hopkins (michael-hopkins@outlook.com)

  References
  ----------
  "Reinforcement Learning of Motor Skills in High Dimensions:
  A Path Integral Approach", ICRA2010

  "A Generalized Path Integral Control Approach to Reinforcment 
  Learning", JMLR2010

--]]

pi2 = {}
pi2.policy = {}
pi2.learner = {}
pi2.dmp_policy = {}
pi2.rmp_policy = {}
pi2.rollout = {}

pi2.policy.__index = pi2.policy
pi2.learner.__index = pi2.learner
pi2.dmp_policy.__index = pi2.dmp_policy
pi2.rmp_policy.__index = pi2.rmp_policy

pi2.policy.__mtstring = 'pi2.policy'
pi2.learner.__mtstring = 'pi2.learner'
pi2.dmp_policy.__mtstring = 'pi2.dmp_policy'
pi2.rmp_policy.__mtstring = 'pi2.rmp_policy'

-- utilities 
--------------------------------------------------------------------------------

local function print_matrix(m)
  print(m:pretty())
  print()
end

local function copy_matrix(ms)
  local md
  if (numlua.type(ms) == 'matrix') then
    md = ms:copy()
  else
    md = matrix.fromtable(ms)
  end
  return md
end

local function row_vector(m)
  return m:reshape(m:size(1), 1) 
end

local function compute_cost_to_go(step_costs, terminal_cost)
   local cost = terminal_cost
   for i = 1,#step_costs do
     cost = cost + step_costs[i]
   end
  return cost
end

-- pi2 rollout
--------------------------------------------------------------------------------

function pi2.rollout.new(n_dimensions, n_time_steps, parameters)
  -- struct for storing trajectory information associated with a single trial
  -- n_dimensions      :  number of dimensions
  -- n_time_steps      :  number of time_steps
  -- parameters        :  parameter vectors (n_dimensions x n_parameters table)

  local o = {}
  o.n_dimensions      = n_dimensions               -- number of dimensions
  o.n_time_steps      = n_time_steps               -- number of time steps
  o.n_parameters      = {}                         -- number of parameters
  o.parameters        = {}                         -- parameter vectors
  o.noise             = {}                         -- noise vectors
  o.projected_noise   = {}                         -- projected noise M(t)*eps
  o.cost              = 0                          -- total cost-to-go
  o.terminal_cost     = 0                          -- terminal cost phi(t)
  o.step_costs        = matrix.zeros(n_time_steps) -- step costs q(t)
  o.path_costs        = matrix.zeros(n_time_steps) -- cumulative path costs S(t)
  o.probabilities     = matrix.zeros(n_time_steps) -- probabilities P(t)

  for d = 1, o.n_dimensions do
    o.n_parameters[d]         = #parameters[d]
    o.parameters[d]           = copy_matrix(parameters[d])
    o.noise[d]                = matrix.zeros(o.n_parameters[d])
    o.projected_noise[d]      = {}

    for i = 1, o.n_time_steps do
      o.projected_noise[d][i] = matrix.zeros(o.n_parameters[d])
    end
  end

  return o
end

-- pi2 learner
--------------------------------------------------------------------------------

function pi2.learner.new(policy, noise_factor, n_rollouts, n_reused_rollouts)
  local o = {}
  o.policy               = policy                 -- pi2.policy object
  o.n_dimensions         = policy:get_n_dimensions() -- number of dimensions
  o.n_time_steps         = policy:get_n_time_steps() -- number of time steps
  o.n_parameters         = {}                     -- number of parameters per dim
  o.n_rollouts           = n_rollouts or 10       -- number of evaluated rollouts
  o.n_reused_rollouts    = n_reused_rollouts or 5 -- number of reused rollouts
  o.iteration            = 1                      -- policy improvement iteration
  o.cost_curve           = {}                     -- policy improvement cost curve
  o.rollouts             = {}                     -- trajectory rollouts
  o.rollouts_initialized = false                  -- are rollouts initialized?
  o.reevealute_rollouts  = false                  -- reevaluate reused rollouts?
  o.parameters           = {}                     -- parameter vectors
  o.basis_vectors        = {}                     -- basis vectors g(t)
  o.temporal_weights     = {}                     -- temporal weights w(t)
  o.projection_matrices  = {}                     -- projection matrices M(t)
  o.noise_factor         = noise_factor or 0.05   -- noise scaling factor
  o.noise_decay_factor   = 1                      -- noise decay factor
  o.noise_variances      = {}                     -- noise variance matrices R
  o.noise_variances_inv  = {}                     -- inverse of R
  o.noise_variances_chol = {}                     -- Cholskey decomposition of R

  local noise_variances  = policy:get_noise_variances()
  local parameters       = policy:get_parameters()
  local basis_vectors    = policy:get_basis_vectors()
  local temporal_weights = policy:get_temporal_weights()

  for d = 1, o.n_dimensions do
    o.n_parameters[d]          = #parameters[d]
    o.parameters[d]            = copy_matrix(parameters[d])
    o.noise_variances[d]       = copy_matrix(noise_variances[d])
    o.noise_variances_inv[d]   = o.noise_variances[d]:inv('P')
    o.noise_variances_chol[d]  = o.noise_variances[d]:chol('L')
    o.basis_vectors[d]         = {}
    o.temporal_weights[d]      = {}

    for i = 1, o.n_time_steps do 
      o.basis_vectors[d][i]    = copy_matrix(basis_vectors[d][i])
      o.temporal_weights[d][i] = copy_matrix(temporal_weights[d][i])
    end
  end

  for k = 1, o.n_rollouts do
    o.rollouts[k] = pi2.rollout.new(o.n_dimensions, o.n_time_steps, parameters)
  end

  pi2.learner.normalize_temporal_weights(o)
  pi2.learner.compute_projection_matrices(o)
  return setmetatable(o, pi2.learner)
end

function pi2.learner.get_parameters(o)
  -- get current parameters
  return o.parameters
end

function pi2.learner.get_rollouts(o)
  -- get current trajectory rollouts
  return o.rollouts
end

function pi2.learner.get_cost_curve(o)
  -- get current cost curve (indexed by policy improvement iteration)
  return o.cost_curve
end

function pi2.learner.get_policy(o)
  -- get pi2 policy reference
  return o.policy
end

function pi2.learner.set_reevaluate_rollouts(o, bool)
  -- set option to reevaluate reused rollouts
  o.reevealute_rollouts = bool
end

function pi2.learner.set_noise_factor(o, noise_factor)
  -- set noise scaling factor
  o.noise_factor = noise_factor
end

function pi2.learner.set_noise_decay(o, noise_decay_factor)
  -- set noise decay factor in the range (0, 1)
  o.noise_decay_factor = noise_decay_factor
end

function pi2.learner.initialize_parameters(o, parameters)
  -- utility function
  for d = 1, o.n_dimensions do
    o.parameters[d] = copy_matrix(parameters[d])
  end
end

function pi2.learner.initialize_noise_variances(o, noise_variances)
  -- utility function
  for d = 1, o.n_dimensions do
    o.noise_variances[d] = copy_matrix(noise_variances[d])
    o.noise_variances_inv[d] = o.noise_variances[d]:inv('P')
    o.noise_variances_chol[d] = o.noise_variances[d]:chol('L')
  end
end

function pi2.learner.initialize_rollouts(o, rollouts)
  -- utility function
  for k = 1, o.n_rollouts do
    o.rollouts = rollouts[i]
  end
  o.rollouts_initialized = true
end

function pi2.learner.generate_noisy_rollouts(o)
   -- generate noisy rollouts for policy improvement
   local index = o.rollouts_initialized and (1 + o.n_reused_rollouts) or 1 

   -- sort current rollouts by total cost (reuse the lowest cost trajectories)
   table.sort(o.rollouts, function (a, b) return a.cost < b.cost end)

   -- generate new noisy rollouts
   for k = index, o.n_rollouts do

     -- add multivariate gaussian noise to the current parameter vector
     local r = o.rollouts[k]
     for d = 1, o.n_dimensions do
       local Rchol = o.noise_variances_chol[d]*math.sqrt(o.noise_factor)
       rng.rmvnorm(matrix.zeros(#r.noise[d]), Rchol, r.noise[d])
       r.parameters[d] = o.parameters[d] + r.noise[d]
     end

     -- evaluate the policy using the noisy parameters and record the costs
     local step_costs, terminal_cost = o.policy:evaluate(r.parameters, false)
     r.step_costs = copy_matrix(step_costs)
     r.terminal_cost = terminal_cost
     r.cost = compute_cost_to_go(step_costs, terminal_cost)
   end

   -- update noise vectors for reused rollouts
   for k = 1, (index - 1) do
     local r = o.rollouts[k]
     for d = 1, o.n_dimensions do 
       r.noise[d]  = r.parameters[d] - o.parameters[d]
     end
   end

   -- reevaluate reused rollouts if needed
   if (o.reevaluate_rollouts) then
     for k = 1, (index - 1) do
       local r = o.rollouts[k]
       local step_costs, terminal_cost = o.policy:evaluate(r.parameters, false)
       r.step_costs = copy_matrix(step_costs)
       r.terminal_cost = terminal_cost
       r.cost = compute_cost_to_go(step_costs, terminal_cost)
     end
   end

   o.rollouts_initialized = true
end

function pi2.learner.normalize_temporal_weights(o)
  -- normalize temporal weights for parameter update averaging
  for d = 1, o.n_dimensions do
    local sum = matrix.zeros(o.n_parameters[d])
    for i = 1, o.n_time_steps do
      sum = sum + o.temporal_weights[d][i]
    end
    for i = 1, o.n_time_steps do
      o.temporal_weights[d][i] = o.temporal_weights[d][i]:div(sum)
    end
  end
end

function pi2.learner.compute_projection_matrices(o)
  -- compute projection matrices M(t) (independent of rollouts)
  for d = 1, o.n_dimensions do

    local Rinv = o.noise_variances_inv[d]/o.noise_factor
    o.projection_matrices[d] = {}

    for i = 1, o.n_time_steps do
      local g = row_vector(o.basis_vectors[d][i])
      local denominator = (g:transpose()*Rinv*g)[1][1]
      o.projection_matrices[d][i] = (Rinv*g*g:transpose())/denominator
    end
  end
end

function pi2.learner.compute_generalized_path_costs(o)
  -- compute generalized path costs S(t) for each rollout
  for k = 1,o.n_rollouts do
    local r = o.rollouts[k]

    -- set the path costs equal to the step costs
    r.path_costs:set(r.step_costs)

    -- add the terminal cost at the final timestep
    r.path_costs[o.n_time_steps] = r.path_costs[o.n_time_steps] + r.terminal_cost

    -- add the control costs for each dimension
    for d = 1, o.n_dimensions do
      local R = o.noise_variances[d]*o.noise_factor
      local M = o.projection_matrices[d]
      local eps = row_vector(r.noise[d])
      local theta = row_vector(o.parameters[d])

      -- note : we assume conditional independence between control dimensions
      for i = 1, o.n_time_steps do
        local projected_noise = M[i]*eps
        local projected_theta = theta + projected_noise
        local control_cost = 
          ((projected_theta:transpose()*R*projected_theta)[1][1])/2
        r.path_costs[i] = r.path_costs[i] + control_cost
        r.projected_noise[d][i] = projected_noise
      end
    end
    
     -- compute cumlative sum over the path costs
     for i = (o.n_time_steps - 1), -1, 1 do
        r.path_costs[i] =  r.path_costs[i] + r.path_costs[i + 1]
     end
  end
end

function pi2.learner.compute_rollout_probabilities(o)
   -- compute trajectory probabilities P(t) for each rollout
   for i = 1, o.n_time_steps do

     local total_probability = 0
     local max_cost = -math.huge
     local min_cost = math.huge

     -- calculate minimum and maximum path cost for the current time-step
     for k = 1, o.n_rollouts do 
       local r = o.rollouts[k]
       if (r.path_costs[i] < min_cost) then
	 min_cost = r.path_costs[i]
       end
       if (r.path_costs[i] > max_cost) then
	 max_cost = r.path_costs[i]
       end
     end

     -- compute P(t) as the exponential of the path cost for each rollout
     for k = 1, o.n_rollouts do
       local r = o.rollouts[k]
       local P = math.exp(-10*(r.path_costs[i] - min_cost)/(max_cost - min_cost))
       r.probabilities[i] = P
       total_probability = total_probability + P
     end

     -- normalize the probabilities
     for k = 1, o.n_rollouts do
       local r = o.rollouts[k]
       r.probabilities[i] = r.probabilities[i]/total_probability
     end
   end
end

function pi2.learner.compute_parameter_updates(o)
   -- compute the new policy parameters using the PI^2 update rule
   local delta_parameters = {}
   for d = 1, o.n_dimensions do 
     delta_parameters[d] = matrix.zeros(o.n_parameters[d])
   end

   -- calculate the parameter deltas for each dimension by integrating over 
   -- the rollouts and averaging over time using the temporal weighting vectors
   for d = 1, o.n_dimensions do
     for k = 1, o.n_rollouts do 
       local r = o.rollouts[k]
       for i = 1, o.n_time_steps do
         delta_parameters[d] = delta_parameters[d] + r.probabilities[i]
           * r.projected_noise[d][i]:mul(o.temporal_weights[d][i])
       end
     end
   end

   -- update the policy parameters
   for d = 1, o.n_dimensions do
     o.parameters[d] = o.parameters[d] + delta_parameters[d]
   end
end

function pi2.learner.compute_trajectory_cost(o)
   -- generate one noiseless rollout to evaluate current trajectory cost
   return compute_cost_to_go(o.policy:evaluate(o.parameters, true))
end

function pi2.learner.improve_policy(o)
  -- improve the policy parameters by evaluating k noisy rollouts
  o:generate_noisy_rollouts()
  o:compute_generalized_path_costs()
  o:compute_rollout_probabilities()
  o:compute_parameter_updates()

  -- evaluate the current cost by evaluating one noiseless rollout
  local cost = o:compute_trajectory_cost()

  -- update learning parameters
  o.noise_factor = o.noise_factor * o.noise_decay_factor
  o.cost_curve[o.iteration] = cost
  o.iteration = o.iteration + 1
  return cost
end

-- pi2 policy
--------------------------------------------------------------------------------

function pi2.policy.new(n_dimensions, n_time_steps)
  -- abstract base class for a generic parameterized control policy
  local o = {}
  o.n_dimensions = n_dimensions                    -- number of dimensions
  o.n_time_steps = n_time_steps                    -- number of time steps
  return setmetatable(o, pi2.policy)
end

function pi2.policy.get_n_dimensions(o)
  return o.n_dimensions
end

function pi2.policy.get_n_time_steps(o)
  return o.n_time_steps
end

function pi2.policy.get_noise_variances(o)
  -- abstract function  : get default controller noise variances
  -- return noise_variances (n_dimensions x n_parameters x n_parameters table)
end

function pi2.policy.get_parameters(o)
  -- abstract function  : get controller parameter vectors
  -- return parameters (n_dimensions x n_parameters table)
end

function pi2.policy.get_basis_vectors(o)
  -- abstract function  : get controller basis vectors
  -- return basis_vectors (n_dimensions x n_time_steps x n_parameters table)
end

function pi2.policy.get_temporal_weights(o)
  -- abstract function  : get temporal weights for parameter update averaging
  -- return temporal_weights (ndimensions x n_time_steps x n_parameters table)
end

function pi2.policy.evaluate(o, parameters, noiseless)
  -- abstract function  :  evaluate the policy and return costs
  -- return step_costs (n_time_steps x 1 table), terminal_cost (double value)
end

-- pi2 dmp policy
--------------------------------------------------------------------------------

function pi2.dmp_policy.new(dmp_object, n_time_steps, dimensions)
  local o = {}
  o.dmp = dmp_object                               -- dmp object
  o.n_time_steps = n_time_steps                    -- number of time_steps
  o.n_dimensions = nil                             -- number of open dimensions
  o.dimensions = {}                                -- open dimensions

  if (dimensions) then
    o.n_dimensions = #dimensions
    for d = 1, o.n_dimensions do
      o.dimensions[d] = dimensions[d]
    end
  else
    o.n_dimensions = o.dmp:get_dimensions() 
    for d = 1, o.n_dimensions do
      o.dimensions[d] = d
    end
  end

  return setmetatable(o, pi2.dmp_policy)
end

function pi2.dmp_policy.get_n_dimensions(o)
  return o.n_dimensions
end

function pi2.dmp_policy.get_n_time_steps(o)
  return o.n_time_steps
end

function pi2.dmp_policy.get_noise_variances(o)
  -- get default controller noise variances
  local noise_variances = {}
  for d = 1, o.n_dimensions do
    local parameters = o.dmp:get_parameters(o.dimensions[d])
    noise_variances[d] = {}
    for i = 1, #parameters do
      noise_variances[d][i] = {}
      for j = 1, #parameters do
        -- set to 0.001*I
        if (i == j) then
          noise_variances[d][i][j] = 0.001
        else
          noise_variances[d][i][j] = 0
        end
      end
    end
  end
  return noise_variances
end

function pi2.dmp_policy.get_parameters(o)
  -- get controller parameter vectors
  local parameters = {}
  for d = 1, o.n_dimensions do
    parameters[d] = o.dmp:get_parameters(o.dimensions[d])
  end
  return parameters
end

function pi2.dmp_policy.get_basis_vectors(o)
  -- get controller basis vectors (assumes dmp time step is set to correct value)
  local basis_vectors = {}
  for d = 1, o.n_dimensions do
    basis_vectors[d] = {}
  end

  -- integrate dmp to get basis vectors at each time step
  o.dmp:reset()
  for i = 1, o.n_time_steps do
    for d = 1, o.n_dimensions do
      basis_vectors[d][i] = o.dmp:get_basis_vector(o.dimensions[d])
    end
    o.dmp:integrate()
  end
  o.dmp:reset()
  
  return basis_vectors
end

function pi2.dmp_policy.get_temporal_weights(o)
  -- get temporal weights for parameter update averaging
  return o:get_basis_vectors() -- use basis activations for temporal weighting
end

function pi2.dmp_policy.evaluate(o, parameters, noiseless)
  -- abstract function  :  evaluate the policy and return costs
  -- return step_costs (n_time_steps x 1 table), terminal_cost (double value)
end


-- pi2 rmp policy
--------------------------------------------------------------------------------

function pi2.rmp_policy.new(rmp_object, n_time_steps, dimensions)
  local o = {} 
  o.rmp = rmp_object                               -- rmp object
  o.n_time_steps = n_time_steps                    -- number of time steps
  o.n_dimensions = nil                             -- number of open dimensions
  o.dimensions = {}                                -- open dimensions

  if (dimensions) then
    o.n_dimensions = #dimensions
    for d = 1, o.n_dimensions do
      o.dimensions[d] = dimensions[d]
    end
  else
    o.n_dimensions = o.rmp:get_dimensions() 
    for d = 1, o.n_dimensions do
      o.dimensions[d] = d
    end
  end

  return setmetatable(o, pi2.rmp_policy)
end

function pi2.rmp_policy.get_n_dimensions(o)
  return o.n_dimensions
end

function pi2.rmp_policy.get_n_time_steps(o)
  return o.n_time_steps
end

function pi2.rmp_policy.get_noise_variances(o)
  -- get default controller noise variances
  local noise_variances = {}
  for d = 1, o.n_dimensions do
    local parameters = o.rmp:get_parameters(o.dimensions[d])
    noise_variances[d] = {}
    for i = 1, #parameters do
      noise_variances[d][i] = {}
      for j = 1, #parameters do
        -- set to 0.001*I
        if (i == j) then
          noise_variances[d][i][j] = 0.001
        else
          noise_variances[d][i][j] = 0
        end
      end
    end
  end
  return noise_variances
end

function pi2.rmp_policy.get_parameters(o)
  -- get controller parameter vectors
  local parameters = {}
  for d = 1, o.n_dimensions do
    parameters[d] = o.rmp:get_parameters(o.dimensions[d])
  end
  return parameters
end

function pi2.rmp_policy.get_basis_vectors(o)
  -- get controller basis vectors (assumes rmp time step is set to correct value)
  local basis_vectors = {}
  for d = 1, o.n_dimensions do
    basis_vectors[d] = {}
  end

  -- integrate rmp to get basis vectors at each time step
  o.rmp:reset()
  for i = 1, o.n_time_steps do
    for d = 1, o.n_dimensions do
      basis_vectors[d][i] = o.rmp:get_basis_vector(o.dimensions[d])
    end
    o.rmp:integrate()
  end
  o.rmp:reset()
  
  return basis_vectors
end

function pi2.rmp_policy.get_temporal_weights(o)
  -- get temporal weights for parameter update averaging
  return o:get_basis_vectors() -- use basis activations for temporal weighting
end

function pi2.rmp_policy.evaluate(o, parameters, noiseless)
  -- abstract function  :  evaluate the policy and return costs
  -- return step_costs (n_time_steps x 1 table), terminal_cost (double value)
end

return pi2
