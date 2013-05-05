-- Torch/Lua Kalman Filter test script
-- (c) 2013 Stephen McGill

local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local libBallTrack = require 'libBallTrack'
-- set the seed
math.randomseed(1234)
local nIter = 30;

-- Initialize the tracker
local tracker = libBallTrack.new_tracker()
-- Set the observations memory segment
local true_positions = torch.Tensor(nIter,2):zero()
local x,P = tracker:get_state()

-- Begin the test loop
for i=1,nIter do

	-- Update the tracker
	local observation = true_positions:select(1,i)
	-- TODO: Add noise to the observation
	-- Remove observations of the ball to ensure the velocity
	-- updates the state, and that the velocity decays over time
	if i/nIter>.5 then
		observation = nil;
	end
	local position, velocity, confidence = tracker:update(observation)

	-- Print debugging information
	x,P = tracker:get_state()
	-- Print the observation of the ball
	if observation then
		observation_str = 'Observe:\t'
		for d=1,observation:size(1) do
			observation_str = observation_str..string.format(' %f',observation[d])
		end
	else
		observation_str = 'Cannot see ball!'
	end
	-- Print our estimation of the state
	state_str = 'State:\t'
	for d=1,x:size(1) do
		state_str = state_str..string.format(' %f',x[d])
	end
	print('Iteration',i)
	print(observation_str)
	print(state_str)
end