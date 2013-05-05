-- Torch/Lua Kalman Filter test script
-- (c) 2013 Stephen McGill

local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local libBallTrack = require 'libBallTrack'
-- Customize the trial
math.randomseed(1234)
torch.manualSeed(1234)
local nIter = 30;
local add_noise = false
local add_roll = false
local pos_noise = 0.05

-- Initialize the tracker
local tracker = libBallTrack.new_tracker()
local reset = true
-- Set the observations (linear movement)
local x1,x2 = 1,.5
local y1,y2 = -.5,.5
local true_x = torch.range(x1,x2,(x2-x1)/nIter):resize(nIter)
local true_y = torch.range(y1,y2,(y2-y1)/nIter):resize(nIter)
local true_dist = torch.sqrt( torch.pow(true_x,2):add(torch.pow(true_y,2)) )
local noise_x = torch.randn(nIter):mul( pos_noise/2 )
local noise_y = torch.randn(nIter):mul( pos_noise/2 )
local pos_tolerance = torch.Tensor(nIter):copy(true_dist):div(20)
local vel_tolerance = torch.Tensor(nIter):copy(true_dist):div(5)

-- Begin the test loop
for i=2,nIter do

	-- Grab the ground truth estimate
	local true_vel = { true_x[i]-true_x[i-1], true_y[i]-true_y[i-1]}
	-- Make the observation
	local observation = {true_x[i],true_y[i]}
	-- Add noise to the observation
	if add_noise then
		observation[1] = observation[1] + noise_x[i]
		observation[2] = observation[2] + noise_y[i]
	end
	-- Remove observations of the ball to ensure the velocity
	-- updates the state, and that the velocity decays over time
	if add_roll and i/nIter>.75 then
		observation = nil;
	end
	-- Update the tracker
	local position, velocity, confidence = tracker:update(observation,reset)
	reset = false
	local pos_error = {true_x[i]-position[1],true_y[i]-position[2]}
	local vel_error = {true_vel[1]-velocity[1],true_vel[2]-velocity[2]}
	-- Make into percentages
	pos_error[1] = pos_error[1] / pos_tolerance[i] * 100
	pos_error[2] = pos_error[2] / pos_tolerance[i] * 100
	vel_error[1] = vel_error[1] / vel_tolerance[i] * 100
	vel_error[2] = vel_error[2] / vel_tolerance[i] * 100

	-- Print the observation of the ball
	
	if observation then
		observation_str = string.format('Observe %d:\t%f %f',
		i, unpack(observation) )
	else
		observation_str = string.format('Observe %d:\tCannot see ball!', i )
	end
	observation_str = observation_str..'\tDistance: '..true_dist[i]
	-- Print our estimation of the state
	position_str = string.format('Position:\t%f, %f', unpack(position) )
	velocity_str = string.format('Velocity:\t%f, %f', unpack(velocity) )
	-- Show the true position of the ball
	true_pos_str = string.format('True Pos:\t%f, %f', true_x[i], true_y[i] )
	true_vel_str = string.format('True Vel:\t%f, %f', unpack(true_vel) )
	-- Show the estimation error
	pos_error_str = string.format('Pos Error:\t%3.4f%%, %3.4f%%', unpack(pos_error) )
	vel_error_str = string.format('Vel Error:\t%3.4f%%, %3.4f%%', unpack(vel_error) )
	tolerance_str = string.format('Tolerance:\t%3.4f\t\t\t%3.4f', 
	pos_tolerance[i], vel_tolerance[i] )
	-- Show the state confidence
	--local pos_confidence = confidence:sub(1,2,1,2):det()
	--pos_confidence_str = string.format('Pos Confidence:\t%f', pos_confidence )
	
	-- Do the printing
	--print(observation_str)
	--print(true_pos_str,true_vel_str)
	--print(position_str,velocity_str)
	--print(pos_error_str,vel_error_str)
	--print(tolerance_str)
	--print(pos_confidence_str)
	--print()
	print(position_str,true_pos_str,velocity_str,true_vel_str)
end