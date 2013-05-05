-- Torch/Lua Ball tracking using a Kalman Filter
-- (c) 2013 Stephen McGill
local libKalman = require'libKalman'
local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

local libBallTrack = {}
local tmp_rotatation = torch.Tensor(2,2)
local MIN_ERROR_DISTANCE = 0.05; -- 5cm
-- TODO: Tune these values...
local ERROR_DEPTH_FACTOR = 0.08;
local ERROR_ANGLE_FACTOR = 3*math.pi/180;
--[[
-- From old BallModel
const double MIN_ERROR_DISTANCE = 50;
const double ERROR_DEPTH_FACTOR = 0.08;
const double ERROR_ANGLE = 3*PI/180;
  static BallModel bm; // Keep the model of the ball static

  double xObject = lua_tonumber(L, 1);
  double yObject = lua_tonumber(L, 2);
  double uncertainty = lua_tonumber(L, 3);

  double distance = sqrt(xObject*xObject+yObject*yObject);
  double angle = atan2(-xObject, yObject);

  double errorDepth = ERROR_DEPTH_FACTOR*(distance+MIN_ERROR_DISTANCE);
  double errorAzimuthal = ERROR_ANGLE*(distance+MIN_ERROR_DISTANCE);

  Gaussian2d objGaussian;
  objGaussian.setMean(xObject, yObject);
  objGaussian.setCovarianceAxis(errorAzimuthal, errorDepth, angle);

  bm.BallObservation(objGaussian, (int)(uncertainty));
--]]

local function rotate_uncertainty( uncertainty, theta )
  -- Homogeneous transformation representing a rotation of theta
  -- about the Z axis.
  local ct = math.cos(theta);
  local st = math.sin(theta);
  local r = torch.eye(2);
  r[1][1] = ct;
  r[2][2] = ct;
  r[1][2] = -1*st;
  r[2][1] = st;
	
	-- Rotate
	tmp_rotatation:mm(r,uncertainty)
	uncertainty:copy(tmp_rotatation)
	
  return uncertainty
end

-- Position filter
-- Yields velocity as well
-- http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5298809
local function customize_filter( filter, nDim )
	-----------------
	-- Ball tracking Parameters
	-----------------
	filter.decay = .95
	filter.dt = 1 -- How many frames have evolved?

	-----------------
	-- Modify the Dynamics update
	-----------------
	filter.A:zero()
	-- Position stays the same
	filter.A:sub(1,nDim,  1,nDim):eye(nDim)
	-- TODO: blocks of the matrix may be mixed up...
	-- Predict next position by velocity
	filter.A:sub(1,nDim, nDim+1,2*nDim):eye(nDim):mul(filter.dt)
	--filter.A:sub(nDim+1,2*nDim, 1,nDim):eye(nDim):mul(filter.dt)
	-- Velocity Decay
	filter.A:sub(nDim+1,2*nDim, nDim+1,2*nDim):eye(nDim):mul(filter.decay)

	-----------------
	-- Modify the Measurement update
	-----------------
	-- We only measure the state positions, not velocities
	filter.R = torch.eye( nDim )
	filter.R[1][1] = 0.01
	filter.H = torch.Tensor( nDim, 2*nDim ):zero()
	filter.H:sub(1,nDim,1,nDim):eye(nDim)

	return filter

end

-- Update the Ball Tracker based on an observation
-- If no positions are given, it is assumed that 
-- we missed an observation during that camera frame
-- Arguments
-- positions: torch.Tensor(2) or {x,y}
local function update( filter, positions )
	-- First, perform prediction
	filter:predict()
	-- Update process confidence based on ball velocity
	--filter.Q
	local state, uncertainty = filter:get_prior()
	-- Next, correct prediction, if positions available
	if positions then
		-- Update measurement confidence
		local x = positions[1]
		local y = positions[2]
		local r = math.sqrt( x^2 + y^2 )
		local theta = math.atan2(-x,y)
		filter.R:eye(2)
		filter.R[1][1] = ERROR_DEPTH_FACTOR*(r+MIN_ERROR_DISTANCE)
		filter.R[2][2] = ERROR_ANGLE_FACTOR*(r+MIN_ERROR_DISTANCE)
		filter.R = rotate_uncertainty( filter.R, theta )
		-- This Tensor instantiation is to support tables or tensors
		-- Tables will be used in regular lua files
		-- We wish to support non-torch programs
		filter:correct( torch.Tensor(positions) )
		state, uncertainty = filter:get_state()
	end
	-- Return position, velocity, uncertainty
	return {state[1],state[2]}, {state[3],state[4]}, uncertainty
end

-- FOR NOW - ONLY USE 2 DIMENSIONS
-- 4 states: x y vx vy
libBallTrack.new_tracker = function()
	local f = {}
	-- Generic filter to start with 2 states per dimension
	f = libKalman.initialize_filter( f, 4 )
	f = customize_filter( f, 2 )
	f.update = update
	f = libKalman.initialize_temporary_variables( f )
	return f
end

-- Arbitrary # of dimensions for generic position tracker
libBallTrack.new_position_filter = function(nDim)
	local f = {}
	-- Generic filter to start with 2 states per dimension
	f = libKalman.initialize_filter( f, 2*nDim )
	f = customize_filter( f, nDim )
	f = libKalman.initialize_temporary_variables( f )
	return f
end

return libBallTrack