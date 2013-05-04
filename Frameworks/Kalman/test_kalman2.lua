local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local kalman1 = require 'libKalman'
local show_kalman_gain = false
local debug_each_state = false

-- 3 dimensional kalman filter
local myDim = 10;
local nIter = 1000;
local x,P = kalman1:init_position_filter( myDim )
local initial_str = 'Initial State:\n'
for d=1,x:size(1) do
	initial_str = initial_str..string.format(' %.3f',x[d])
end
print(initial_str)

-- Do not control input
local u_k_input = torch.Tensor( 2*myDim ):zero()

for i=1,nIter do
	-- One Kalman
	x,P = kalman1:predict( u_k_input )
	local prior_str = 'Prior:\t'
	for d=1,x:size(1) do
		prior_str = prior_str..string.format(' %f',x[d])
	end
	
	-- Add Observations
	local obs = torch.Tensor(myDim):zero()
	obs[1] = i + .2*(math.random()-.5)
	for p=2,obs:size(1)-1 do
		obs[p] = i/p + 1/(5*p)*(math.random()-.5)
	end
	
	local observation_str = 'Observe:\t'
	for d=1,obs:size(1) do
		observation_str = observation_str..string.format(' %f',obs[d])
	end
	x,P,K = kalman1:correct( obs )
	local state_str = 'State:\t'
	for d=1,x:size(1) do
		state_str = state_str..string.format(' %f',x[d])
	end
	-- Debug the Gain
	local kgain_str = ''
	if show_kalman_gain then
		kgain_str = 'Kalman gain\n'
		for i=1,K:size(1) do
			for j=1,K:size(2) do
				kgain_str = kgain_str..string.format('   %f',K[i][j])
			end
			kgain_str = kgain_str..'\n'
		end
	end
	-- End Debug of gain
	-- Debug of A
	local a_str = ''
	if show_kalman_gain then
		a_str = 'A\n'
		local A = kalman1.A;
		for i=1,A:size(1) do
			for j=1,A:size(2) do
				a_str = a_str..string.format('   %.3f',A[i][j])
			end
			a_str = a_str..'\n'
		end
	end
	-- End Debug of gain
	
	if debug_each_state then
		print('Observation',i)
		print(prior_str)
		print(observation_str)
		print(state_str)
		print(a_str)
		print(kgain_str)
	end
	
end

local final_str = 'Final State:\n'
for d=1,x:size(1) do
	final_str = final_str..string.format(' %.3f',x[d])
end
print(final_str)