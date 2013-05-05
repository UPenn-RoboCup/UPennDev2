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
-- Set the observations (linear movement)
local x1,x2 = 1,.5
local y1,y2 = -.5,.5
local true_x = torch.range(x1,x2,(x2-x1)/nIter):resize(nIter)
local true_y = torch.range(y1,y2,(y2-y1)/nIter):resize(nIter)

-- Begin the test loop
for i=1,nIter do

	-- Update the tracker
	local observation = {true_x[i],true_y[i]}
	-- TODO: Add noise to the observation
	-- Remove observations of the ball to ensure the velocity
	-- updates the state, and that the velocity decays over time
	if i/nIter>.5 then
		observation = nil;
	end
	local position, velocity, confidence = tracker:update(observation)

	-- Print the observation of the ball
	if observation then
		observation_str = string.format('Observe %d:\t%f %f',i,unpack(observation))
	else
		observation_str = string.format('Observe %d:\tCannot see ball!', i )
	end
	-- Print our estimation of the state
	position_str = string.format('Position:\t%f %f', unpack(position) )
	velocity_str = string.format('Velocity:\t%f %f', unpack(velocity) )
	-- Show the true position of the ball
	true_str = string.format('True Value:\t%f %f', true_x[i], true_y[i] )
	-- Do the printing
	print(observation_str)
	print(true_str)
	--print('\t============')
	print(position_str)
	--print(velocity_str)
	print()
end