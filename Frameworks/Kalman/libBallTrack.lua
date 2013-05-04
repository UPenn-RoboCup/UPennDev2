-- Torch/Lua Ball tracking using a Kalman Filter
-- (c) 2013 Stephen McGill

local libKalman = require'libKalman'
local libBallTrack = {}

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
	-- Position stays the same
	filter.A:sub(1,nDim,  1,nDim):eye(nDim)
	-- Predict next position by velocity
	filter.A:sub(1,nDim, nDim+1,2*nDim):eye(nDim):mul(filter.dt)
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

	-----------------
	-- Modify the initial state prior
	-----------------
	filter.P_k_minus[1][1] = 0.01 -- Trust position
	filter.P_k_minus[2][2] = 0.1  -- Do not trust velocity much
	-- Duplicate for current state...
	filter.P_k[1][1] = 0.01 -- Trust position
	filter.P_k[2][2] = 0.1  -- Do not trust velocity much

	return filter

end

libBallTrack.new_position_filter = function( nDim )
	local f = {}
	-- Generic filter to start with 2 states per dimension
	f = libKalman.initialize_filter( f, 2*nDim )
	f = customize_filter( f, nDim )
	f = libKalman.initialize_temporary_variables( f )
	return f
end

return libBallTrack