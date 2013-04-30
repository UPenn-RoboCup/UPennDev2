local torch = require 'torch'
torch.Tensor = torch.DoubleTensor
local kalman1 = require 'libKalman'

local test_two = false
local show_kalman_gain = false

if test_two then
	kalman2 = require 'libKalman'
end

-- 2 dimensional kalman filter
local myDim = 2;
local x,P = kalman1:init_position_filter( myDim )
--local x,P = kalman1:initialize( myDim )
print('Initial:',x[1],x[2])

for i=1,10 do
	-- One Kalman
	x,P = kalman1:predict()
	print('Prior:', x[1],x[2] )
	x,P,K = kalman1:correct( torch.Tensor(myDim):fill(1) )
	print('State:', x[1],x[2] )
	if show_kalman_gain then
		print('Kalman')
		local str = ''
		for i=1,myDim do
			for j=1,myDim do
				str = str..' '..K[i][j]
			end
			str = str..'\n'
		end
		print(str)
	end
end

if test_two then
	-- 3 dimensional kalman filter
	myDim = 3;
	x,P = kalman2:initialize( myDim )
	print('Initial:',x[1],x[2])

	for i=1,10 do
		-- One Kalman
		x,P = kalman2:predict()
		print('Prior:',x[1],x[2],x[3])
		x,P,K = kalman2:correct( torch.Tensor(myDim):fill(2) )
		print('State:',x[1],x[2],x[3])
	end
end