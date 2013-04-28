local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local kalman1 = require 'libKalman'
local kalman2 = require 'libKalman'

-- 2 dimensional kalman filter
local myDim = 2;
local x,P = kalman1:initialize( myDim )
print('Initial:',x[1],x[2])

for i=1,10 do
	-- One Kalman
	x,P = kalman1:predict()
	print('Prior:',x[1],x[2])
	x,P = kalman1:correct( torch.Tensor(myDim):fill(1) )
	print('State:',x[1],x[2])
end

-- 3 dimensional kalman filter
myDim = 3;
x,P = kalman2:initialize( myDim )
print('Initial:',x[1],x[2])

for i=1,10 do
	-- One Kalman
	x,P = kalman2:predict()
	print('Prior:',x[1],x[2],x[3])
	x,P = kalman2:correct( torch.Tensor(myDim):fill(2) )
	print('State:',x[1],x[2],x[3])
end