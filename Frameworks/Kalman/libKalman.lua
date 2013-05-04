local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

local filter = {}

filter.initialize = function( self, nDim )
	-- Utility
	self.I = torch.eye( nDim )
	-- Process
	self.A = torch.eye(nDim) -- State process w/o input
	self.B = torch.eye(nDim) -- Control input to state effect
	self.Q = torch.eye(nDim) -- Additive uncertainty
	-- Measurement
	self.R = torch.eye(nDim) -- Measurement uncertainty
	self.H = torch.eye(nDim) 
	-- Prior
	self.P_k_minus = torch.eye(nDim)
	self.x_k_minus = torch.Tensor( nDim ):zero()
	-- State
	self.P_k = torch.eye(nDim)
	self.x_k = torch.Tensor( nDim ):zero()
	-- Temporaty variables
	self:init_temporary_variables()
	return self.x_k, self.P_k
end

-- Position filter
-- Yields velocity as well
-- http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5298809
filter.init_position_filter = function( self, nDim )
	-- Ball tracking Parameters
	self.decay = .95
	self.dt = 1 -- How many frames have evolved?
	-- Utility
	self.I = torch.eye( 2*nDim )
	-- Process
	-- State stays the same by default
	self.A = torch.eye(2*nDim)
	-- Position stays the same
	self.A:sub(1,nDim,  1,nDim):eye(nDim)
	-- Predict next position by velocity
	self.A:sub(1,nDim, nDim+1,2*nDim):eye(nDim):mul(self.dt)
	-- Velocity Decay
	self.A:sub(nDim+1,2*nDim, nDim+1,2*nDim):eye(nDim):mul(self.decay)
	-- External input model...
	self.B = torch.eye(2*nDim)
	-- Additive Motion Uncertainty
	self.Q = torch.eye(2*nDim)
	-- Measurement
	self.R = torch.eye( nDim )
	self.R[1][1] = 0.01
	self.H = torch.Tensor( nDim, 2*nDim ):zero()
	self.H:sub(1,nDim,1,nDim):eye(nDim)
	-- Prior
	self.P_k_minus = torch.eye( 2*nDim )
	self.P_k_minus[1][1] = 0.01
	self.P_k_minus[2][2] = 0.1
	self.x_k_minus = torch.Tensor( 2*nDim ):zero()
	-- State
	self.P_k = torch.eye( 2*nDim ):copy( self.P_k_minus )
	self.x_k = torch.Tensor( 2*nDim ):copy( self.x_k_minus )
	-- Temporary vars
	self:init_temporary_variables()
	return self.x_k, self.P_k
end

filter.init_temporary_variables = function(self)
	-- Temporary Variables for memory savings
	self.tmp_input = torch.Tensor( self.B:size(2) )
	--torch.mv( self.B, self.u_k ):zero()
	
	self.tmp_state = torch.mv( self.A, self.x_k_minus ):zero()
	self.tmp_covar = torch.mm( self.A, self.P_k_minus ):zero()
	self.tmp_pcor1 = torch.mm( self.H, self.P_k_minus ):zero()
	self.tmp_pcor2 = torch.mm( self.tmp_pcor1, self.H:t() ):zero()
	self.tmp_pcor3 = torch.mm( self.tmp_pcor1, self.H:t() ):zero()
	self.tmp_pcor4 = torch.mm( self.P_k_minus, self.H:t()):zero()
	self.K_k = torch.mm(self.P_k_minus, self.H:t()):zero()
	self.K_update  = torch.mm( self.K_k, self.H ):zero()
	self.tmp_scor  = torch.mv( self.H, self.x_k_minus ):zero()
end

-- Form a state estimate prior based on the process and input
filter.predict = function(self, u_k)
	-- Complicated (i.e. fast in-memory) way
	self.tmp_input:mv( self.B, u_k )
	self.tmp_state:mv( self.A, self.x_k_minus )
	self.x_k_minus:copy( self.tmp_state ):add( self.tmp_input )
	self.tmp_covar:mm( self.A, self.P_k_minus )
	self.P_k_minus:mm( self.tmp_covar, self.A:t() ):add( self.Q )
	
	--[[
	-- Simple (i.e. mallocing memory each time) way
	self.x_k_minus = self.A * self.x_k_minus + self.B * self.u_k
	self.P_k_minus = self.A * self.P_k_minus * self.A:t() + self.Q
	--]]
	
	return self.x_k_minus, self.P_k_minus
end

-- Correct the state estimate based on the state estimate prior and measurement
filter.correct = function( self, z_k )

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
	return self.x_k, self.P_k, self.K_k
end

return filter