-- Torch/Lua Kalman Filter
-- (c) 2013 Stephen McGill

local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

-- Accessor Methods
local function get_prior(self)
	return self.x_k_minus, self.P_k_minus
end
local function get_state(self)
	return self.x_k, self.P_k
end

-- Form a state estimate prior based on the process and input
local function predict(self, u_k)
	-- Complicated (i.e. fast in-memory) way
	
	self.tmp_input:mv( self.B, u_k )
	self.tmp_state:mv( self.A, self.x_k_minus )
	self.x_k_minus:copy( self.tmp_state ):add( self.tmp_input )
	self.tmp_covar:mm( self.A, self.P_k_minus )
	self.P_k_minus:mm( self.tmp_covar, self.A:t() ):add( self.Q )
	--]]
	
	--[[
	-- Simple (i.e. mallocing memory each time) way
	self.x_k_minus = self.A * self.x_k_minus + self.B * u_k
	self.P_k_minus = self.A * self.P_k_minus * self.A:t() + self.Q
	--]]
end

-- Correct the state estimate based on a measurement
local function correct( self, z_k )
	-- Complicated (i.e. fast in-memory) way

	self.tmp_pcor1:mm( self.H, self.P_k_minus )
	self.tmp_pcor2:mm( self.tmp_pcor1, self.H:t() ):add(self.R)
	torch.inverse( self.tmp_pcor3, self.tmp_pcor2 )
	self.tmp_pcor4:mm(self.P_k_minus, self.H:t() )	
	self.K_k:mm( self.tmp_pcor4, self.tmp_pcor3 )
	self.K_update:mm( self.K_k, self.H ):mul(-1):add(self.I)
	self.P_k:mm( self.K_update, self.P_k_minus )
	self.tmp_scor:mv( self.H, self.x_k_minus ):mul(-1):add(z_k)
	self.x_k:mv( self.K_k, self.tmp_scor ):add(self.x_k_minus)
	--]]
	
	--[[
	-- Simple (i.e. mallocing memory each time) way
	local tmp1 = self.H * self.P_k_minus * self.H:t()
	local tmp = tmp1 + self.R
	self.K_k = self.P_k_minus * self.H:t() * torch.inverse(tmp)
	self.P_k = (self.I - self.K_k * self.H) * self.P_k_minus
	self.x_k = self.x_k_minus + self.K_k * (z_k - self.H * self.x_k_minus)
	--]]

	-- Duplicate Values
	self.x_k_minus:copy(self.x_k)
	self.P_k_minus:copy(self.P_k)
end

-- Filter initialization code
local function initialize_filter( filter, nDim )
	-- Utility
	filter.I = torch.eye(nDim)
	-- Process
	filter.A = torch.eye(nDim) -- State process w/o input
	filter.B = torch.eye(nDim) -- Control input to state effect
	filter.Q = torch.eye(nDim) -- Additive uncertainty
	-- Measurement
	filter.R = torch.eye(nDim) -- Measurement uncertainty
	filter.H = torch.eye(nDim) 
	-- Prior
	filter.P_k_minus = torch.eye(nDim)
	filter.x_k_minus = torch.Tensor(nDim):zero()
	-- State
	filter.P_k = torch.Tensor( nDim, nDim ):copy( filter.P_k_minus )
	filter.x_k = torch.Tensor(nDim):copy( filter.x_k_minus )
	
	----------
	-- Methods
	----------
	filter.predict = predict
	filter.correct = correct
	filter.get_prior = get_prior
	filter.get_state = get_state
	
	return filter
end

-- Temporary Variables for complicated fast memory approach
local function initialize_temporary_variables( filter )
	filter.tmp_input = torch.Tensor( filter.B:size(1) )
	filter.tmp_state = torch.Tensor( filter.A:size(1) )
	filter.tmp_covar = torch.Tensor( filter.A:size(1), filter.P_k_minus:size(2) )
	filter.tmp_pcor1 = torch.Tensor( filter.H:size(1), filter.P_k_minus:size(2) )
	filter.tmp_pcor2 = torch.Tensor( filter.tmp_pcor1:size(1), filter.H:size(1) )
	filter.tmp_pcor3 = torch.Tensor( filter.tmp_pcor1:size(1), filter.H:size(1) )
	filter.tmp_pcor4 = torch.Tensor( filter.P_k_minus:size(1), filter.H:size(1) )
	filter.K_k = torch.Tensor( filter.P_k_minus:size(1), filter.H:size(1) )
	filter.K_update = torch.Tensor( filter.K_k:size(1), filter.H:size(2) )
	filter.tmp_scor  = torch.Tensor( filter.H:size(1) )
	return filter
end

-- Position filter
-- Yields velocity as well
-- http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5298809
local function initialize_position_filter ( filter, nDim )
	-- Generic filter to start
	filter = initialize_filter( filter, 2*nDim )
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

-- Begin the library code
local libKalman = {}

-- Generic filter
libKalman.new_filter = function( nDim )
	local f = {}
	-- Default initialization
	f = initialize_filter( f, nDim )
	f = initialize_temporary_variables( f )
	return f
end

-- Special Position filter
libKalman.new_position_filter = function( nDim )
	local f = {}
	-- Default initialization
	f = initialize_position_filter( f, nDim )
	f = initialize_temporary_variables( f )
	return f
end

return libKalman