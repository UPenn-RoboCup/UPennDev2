local octomap=require'octomap'
local torch=require'torch'
torch.Tensor = torch.DoubleTensor
local unix = require 'unix'

local dist = 5
local nrays = 300
local nz = 5
local myscan = torch.DoubleTensor(nrays,4):zero()
local myorigin = torch.DoubleTensor(3):zero()

for z=1,nz do
	for i=1,nrays do
		local distr = dist+1*(math.random(100)/100-.5)
		myscan[i][1] = distr*math.cos(i/nrays*math.pi)
		myscan[i][2] = distr*math.sin(i/nrays*math.pi)
		myscan[i][3] = z/nz
	end
	if z>nz/2 then
		-- Use raycast
		local origin = torch.Tensor({0,0,z})
		octomap.set_origin( origin )
		octomap.add_scan( myscan, 1 )
	else
		t0 = unix.time()
		local sz = octomap.add_scan( myscan )
		t1 = unix.time()
		print('Add scan',t1-t0,sz)
	end
end

t0 = unix.time()
local tree_str = octomap.get_data()
t1 = unix.time()
print("Got unpruned",#tree_str,t1-t0)

t0 = unix.time()
local tree_str_pruned = octomap.get_pruned_data()
t1 = unix.time()
print("Got pruned",#tree_str_pruned,t1-t0)

octomap.save_tree('demo.bt')
