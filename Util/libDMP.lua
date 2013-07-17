-- Torch/Lua Dynamic Movement Primitives
-- (c) 2013 Stephen McGill

local torch = require 'torch'
torch.Tensor = torch.DoubleTensor

-- Begin the library code
local libDMP = {}

local function basis_function(x,sigma,center)
	sigma = sigma or 1
	center = center or 0
	return math.exp( -1/(2*sigma) * (x-center)^2 )
end

local function forcing_function()

end

libDMP.learn_from_examples = function( trajectories )
	
end