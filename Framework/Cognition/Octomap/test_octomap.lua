local octomap=require 'octomap'
local torch = require 'torch'
torch.Tensor = torch.FloatTensor

local dist = 5
local nrays = 300
local nz = 5
local myscan = torch.FloatTensor(3,nrays):zero()

for z=1,nz do
	for i=1,nrays do
		local distr = dist+1*(math.random(100)/100-.5)
		myscan[1][i] = distr*math.cos(i/nrays*math.pi)
		myscan[2][i] = distr*math.sin(i/nrays*math.pi)
		myscan[3][i] = z/nz
	end
	octomap.add_scan( myscan )
end

octomap.save_tree()
