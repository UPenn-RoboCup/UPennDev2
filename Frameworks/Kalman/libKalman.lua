local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

local filter = {}
filter.decay = .9

filter.initialize = function( self, nDim )
	-- Utility
	self.I = torch.eye( nDim )
	-- Process
	self.A = torch.eye(nDim) * self.decay
	self.B = torch.eye(nDim)
	self.u_k = torch.Tensor( nDim )
	self.Q = torch.eye(nDim)
	-- Measurement
	self.R = torch.eye(nDim)
	self.H = torch.eye(nDim)
	-- Prior
	self.P_k_minus = torch.eye(nDim)
	self.x_k_minus = torch.Tensor( nDim )
	-- State
	self.P_k = torch.eye(nDim)
	self.x_k = torch.Tensor( nDim )
	return self.x_k, self.P_k
end

-- Position filter
-- Yields velocity as well
filter.init_position_filter = function( self, nDim )
	-- Utility
	self.I = torch.eye( 2*nDim )
	-- Process
	self.A = torch.eye(2*nDim) * self.decay
	self.B = torch.eye(2*nDim)
	self.u_k = torch.Tensor( 2*nDim )
	self.Q = torch.eye(2*nDim)
	-- Measurement
	self.R = torch.eye( nDim )
	self.H = torch.Tensor( nDim, 2*nDim )
	self.H:sub(1,nDim,1,nDim):eye(nDim)
	-- Prior
	self.P_k_minus = torch.eye( 2*nDim )
	self.x_k_minus = torch.Tensor( 2*nDim )
	-- State
	self.P_k = torch.eye( 2*nDim )
	self.x_k = torch.Tensor( 2*nDim )
	self.P_k_minus = torch.eye( 2*nDim )
	self.x_k_minus = torch.Tensor( 2*nDim )
	return self.x_k, self.P_k
end

-- Form a state estimate prior based on the process
filter.predict = function(self)
	-- There may be many predictions before a measurement
	self.x_k_minus = self.A * self.x_k_minus + self.B * self.u_k
	self.P_k_minus = self.A * self.P_k_minus * self.A:t() + self.Q
	return self.x_k_minus, self.P_k_minus
end

-- Correct the state estimate based on the state estimate prior and measurement
filter.correct = function( self, z_k )
	local tmp1 = self.H * self.P_k_minus * self.H:t()
	local tmp = tmp1 + self.R
	local K_k = self.P_k_minus * self.H:t() * torch.inverse(tmp)
	self.P_k = (self.I - K_k * self.H) * self.P_k_minus
	self.x_k = self.x_k_minus + K_k * (z_k - self.H * self.x_k_minus)
	-- Prior becomes the corrected state
	self.x_k_minus, self.P_k_minus = self.x_k, self.P_k
	return self.x_k, self.P_k, K_k
end

return filter