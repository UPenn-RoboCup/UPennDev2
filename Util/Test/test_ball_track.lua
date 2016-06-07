-- Torch/Lua Kalman Filter test script
-- (c) 2013 Stephen McGill

dofile'../../include.lua'

local torch = require 'torch'
local vector = require 'vector'
local libBallTrack = require 'libBallTrack'
-- set the seed
math.randomseed(1234)

-- Debugging options
local show_kalman_gain = false
local debug_each_state = false

local observations = {
{1.0572, -0.1281},
{1.0410, -0.1395},
{1.0219, -0.1466},
{1.0148, -0.1580},
{1.0103, -0.1710},
{0.9929, -0.1804},
{0.9683, -0.1883},
{0.9640, -0.1978},
{0.9437, -0.2060},
{0.9336, -0.2162},
{0.9185, -0.2223},
{0.9101, -0.2342},
{0.8953, -0.2396},
{0.8807, -0.2475},
{0.8696, -0.2581},
{0.8590, -0.2652},
{0.8406, -0.2695},
{0.8317, -0.2798},
{0.8208, -0.2894},
{0.8077, -0.2959},
{0.7909, -0.3075},
{0.7761, -0.3394},
{0.7655, -0.3348},
{0.7555, -0.3311},
{0.7477, -0.3346},
{0.7491, -0.3423},
{0.7389, -0.3473},
{0.7341, -0.3546},
{0.7286, -0.3628},
{0.7115, -0.3675},
{0.7028, -0.3726},
{0.6922, -0.3747},
{0.6902, -0.3863},
{0.6796, -0.3903},
{0.6744, -0.3996},
{0.6653, -0.4041},
{0.6604, -0.4107},
{0.6559, -0.4188},
{0.6425, -0.4222},
{0.6390, -0.4291},
{0.6369, -0.4364},
{0.6340, -0.4470},
{0.6240, -0.4484},
{0.6196, -0.4551},
--[[
{0.6196, -0.4653},
{0.6118, -0.4691},
{0.6077, -0.4775},
{0.6086, -0.4851},
{0.5991, -0.4880},
{0.6036, -0.5018},
{0.5950, -0.5037},
{0.5917, -0.5089},
{0.5819, -0.5263},
{0.5541, -0.5526},
{0.5634, -0.5467},
{0.5651, -0.5505},
{0.5725, -0.5481},
{0.5764, -0.5551},
{0.5749, -0.5587},
{0.5690, -0.5584},
{0.5701, -0.5627},
{0.5736, -0.5769},
{0.5686, -0.5754},
{0.5647, -0.5757},
{0.5649, -0.5789},
{0.5681, -0.5877},
{0.5727, -0.6007},
{0.5695, -0.5962},
{0.5649, -0.5933},
{0.5647, -0.6000},
{0.5644, -0.5999},
{0.5695, -0.6081},
{0.5647, -0.6029},
{0.5639, -0.6018},
{0.5705, -0.6109},
{0.5719, -0.6121},
{0.5681, -0.6066},
{0.5658, -0.6014},
{0.5734, -0.6106},
{0.5687, -0.6012},
{0.5660, -0.5997}
--]]
}

-- 3 dimensional kalman filter
local nIter = 5000;
-- Initialize the filter
--local kalman1 = libBallTrack.new_position_filter(#observations[1], observations[1])
local kalman1 = libBallTrack.new_tracker(observations[1])

-- Set the observations
local obs1 = torch.Tensor(observations[1])
local x,P = kalman1:get_state()

-- Print the initial state
local initial_str = 'Initial State:\n'
for d=1,x:size(1) do
	initial_str = initial_str..string.format(' %.3f',x[d])
end
print(initial_str)

local filtered = {}

-- Begin the test loop
for i=1,#observations do

  print('==')
  kalman1:update(observations[i])
  --[[
	-- Perform prediction
	kalman1:predict()
	local x_pred, P_pred = kalman1:get_prior()
	-- Perform correction
	kalman1:correct( torch.Tensor(observations[i]) )
  --]]
  x,P = kalman1:get_state()

  table.insert(filtered, {x:clone(), P:clone()})

	-- Print debugging information
	if debug_each_state then
		
		-- Save prediction string
    if x_pred then
      local prior_str = 'Prior:\t'
  		for d=1,x_pred:size(1) do
  			prior_str = prior_str..string.format(' %f',x_pred[d])
  		end
    end
    if P_pred then
      local prior_unc_str = 'Prior Uncertainty:\n'..tostring(P_pred)
    end
		
		-- Save observation string
		local observation_str = 'Obs:\t '..table.concat(observations[i], ',')
		
		-- Save corrected state string
		local state_str = 'State:\t'
		for d=1,x:size(1) do
			state_str = state_str..string.format(' %f',x[d])
		end
    
    -- Uncertainty
		local unc_str = 'Uncertainty:\n'..tostring(P)
		
		print('==\nIteration',i)
		print(prior_str)
		print(observation_str)
		print(state_str)
    
    print(prior_unc_str)
    print(unc_str)
    
		if show_kalman_gain then
			-- Save the Kalman gain and A strings
			local kgain_str = 'Kalman gain\n'
			local K = kalman1.K_k;
			for i=1,K:size(1) do
				for j=1,K:size(2) do
					kgain_str = kgain_str..string.format('   %f',K[i][j])
				end
				kgain_str = kgain_str..'\n'
			end
			local a_str = 'A:\n'
			local A = kalman1.A;
			for i=1,A:size(1) do
				for j=1,A:size(2) do
					a_str = a_str..string.format('   %.3f',A[i][j])
				end
				a_str = a_str..'\n'
			end
			print(a_str)
			print(kgain_str)
		end
	end
	
end

x,P = kalman1:get_state()
local final_str = 'Final State:\n'
for d=1,x:size(1) do
	final_str = final_str..string.format(' %.3f',x[d])
end
print(final_str)

for i, v in ipairs(filtered) do
  print(vector.new(v[1])/100)
  --print(unpack(v))
end