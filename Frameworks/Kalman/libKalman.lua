local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

local filter = {}
filter.decay = .9

filter.initialize = function( nDim )
	-- Process
	self.A = torch.eye(nDim) * self.decay
	self.B = torch.eye(nDim)
	self.H = torch.eye(nDim)
	-- Measurement
	self.Q = torch.eye(nDim)
	-- State
	self.P_k = torch.eye(nDim)
	self.x_k = torch.Tensor( nDim )
	-- Prior
	self.P_k_minus = torch.eye(nDim)
	self.x_k_minus = torch.Tensor( nDim )
	-- Utility
	self.I = torch.eye( nDim )
end

-- Form a state estimate prior based on the process
filter.predict = function(self)
	self.x_k_minus = self.A * self.x_k + self.B * self.u_k
	self.P_k_minus = self.A * self.P_k * self.A:t() + self.Q
end

-- Correct the state estimate based on the state estimate prior and measurement
filter.correct = function(self)
	local tmp = self.H * self.P_k_minus * self.H:t() + self.R
	local K_k = self.P_k_minus * self.h:t() * tmp:inv()
	self.P_k = (self.I - self.K_k * self.H) * self.P_k_minus
	self.x_k = self.x_k_minus + self.K_k * (self.z_k - self.H*self.x_k_minus)
end

return filter