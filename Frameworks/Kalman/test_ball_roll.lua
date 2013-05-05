-- Torch/Lua Kalman Filter test script
-- (c) 2013 Stephen McGill

local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local libBallTrack = require 'libBallTrack'
-- Customize the trial
math.randomseed(1234)
local nIter = 30;
local add_noise = true
-- 5cm tolerance, 50cm/s tolerance
local pos_tolerance, vel_tolerance = 0.05, 0.5

-- Initialize the tracker
local tracker = libBallTrack.new_tracker()
-- Set the observations (linear movement)
local x1,x2 = 1,.5
local y1,y2 = -.5,.5
local true_x = torch.range(x1,x2,(x2-x1)/nIter)
local true_y = torch.range(y1,y2,(y2-y1)/nIter)

-- Begin the test loop
for i=2,nIter do

	-- Grab the ground truth estimate
	local ground_truth = torch.Tensor( {true_x[i],true_y[i]} )
	local true_vel = { true_x[i]-true_x[i-1], true_y[i]-true_y[i-1]}
	-- Add noise to the observation
	if add_noise then
		local noise = torch.randn(2):mul( pos_tolerance/2 )
		ground_truth:add( noise )
	end
	-- Make the observation
	local observation = {ground_truth[1],ground_truth[2]}
	-- Remove observations of the ball to ensure the velocity
	-- updates the state, and that the velocity decays over time
	if i/nIter>.5 then
		observation = nil;
	end
	-- Update the tracker
	local position, velocity, confidence = tracker:update(observation)
	local pos_error = {true_x[i]-position[1],true_y[i]-position[2]}
	local vel_error = {true_vel[1]-velocity[1],true_vel[2]-velocity[2]}
	-- Make into percentages
	pos_error[1] = pos_error[1] / pos_tolerance * 100
	pos_error[2] = pos_error[2] / pos_tolerance * 100
	vel_error[1] = pos_error[1] / vel_tolerance * 100
	vel_error[2] = pos_error[2] / vel_tolerance * 100

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
	true_pos_str = string.format('True Pos:\t%f %f', true_x[i], true_y[i] )
	true_vel_str = string.format('True Vel:\t%f %f', unpack(true_vel) )
	-- Show the estimation error
	pos_error_str = string.format('Pos Error:\t%.4f %% %.4f %%', unpack(pos_error) )
	vel_error_str = string.format('Vel Error:\t%.3f %% %.3f %%', unpack(vel_error) )
	-- Do the printing
	print(observation_str)
	print(true_pos_str,true_vel_str)
	--print('\t============')
	print(position_str,velocity_str)
	print(pos_error_str,vel_error_str)
	--print(velocity_str)
	print()
end