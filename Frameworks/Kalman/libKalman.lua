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

-- Form a state estimate prior based on the process
filter.predict = function(self)
	self.x_k_minus = self.A * self.x_k + self.B * self.u_k
	self.P_k_minus = self.A * self.P_k * self.A:t() + self.Q
	return self.x_k_minus, self.P_k_minus
end

-- Correct the state estimate based on the state estimate prior and measurement
filter.correct = function( self, z_k )
	local tmp = self.H * self.P_k_minus * self.H:t() + self.R
	local K_k = self.P_k_minus * self.H:t() * torch.inverse(tmp)
	self.P_k = (self.I - K_k * self.H) * self.P_k_minus
	self.x_k = self.x_k_minus + K_k * (z_k - self.H * self.x_k_minus)
	return self.x_k, self.P_k
end

return filter