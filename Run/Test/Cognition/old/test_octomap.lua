require 'Octomap'
require 'torch'
local sz = 1000
local myscan_xyz = torch.FloatTensor(3,sz):fill(0)

for i=1,sz do
  myscan_xyz[2][i] = i/100
  myscan_xyz[3][i] = math.random(3)
end

Octomap.add_scan( myscan_xyz )
Octomap.save_tree()
