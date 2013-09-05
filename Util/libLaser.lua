-- libLaser
-- (c) 2013 Stephen McGill
-- Filtering of the laser points from the device

require 'torch'
torch.Tensor = torch.DoubleTensor
local libTrig = require 'libTrig'
--local tutil = require 'tutil'

local libLaser = {}


local function transform(self, roll, pitch, yaw)
	--------------
	-- TODO: this resize should be unnecessary
	-- Reset the range container size
	self.ranges:resize( self.maxIdx - self.minIdx + 1 )
	self.points:resize( (self.maxIdx - self.minIdx + 1), 4 )
	--------------

	--------------
	-- Easier access to data
	local X = self.points
	local xs = X:select(2,1);
	local ys = X:select(2,2);
	--------------

	--------------
	-- Transform LIDAR readings into relative cartesian coordinate
	
	--print(string.format('\n\nSize of ranges: %d', self.ranges:size(1)))
	--print(string.format('Size of cosines: %d \n\n', self.cosines:size(1)))
	
	xs:copy(self.ranges):cmul( self.cosines )
	ys:copy(self.ranges):cmul( self.sines )
	--------------

	--------------
	--[[ Prune based on the minimum and maximum ranges
	self.nRanges = tutil.band_mask_key(
	self.ranges,
	self.minRange, self.maxRange,
	xs, ys
	)
	--]]

	-- TODO: temporarily no pruning
	self.nRanges = self.ranges:size(1)
	if self.nRanges < 1 then
		return
	end
	X:resize( self.nRanges, 4 )


	--------------

	-- Transformation from body to lidar
	if self.location == 'head' then		
		self:update_head( roll, pitch, yaw ) 
		
		------------------------------------------------
		-- Perform the transformations to the body frame
		self.points_xyz:resize( X:size(1), 4 )
		self.points_xyz:mm( X, self.T:t() )
		------------------------------------------------

		--[[ Prune the lidar points that are outside of proper height
		tutil.band_mask_key_points(
		self.points_xyz:select(2,3),
		self.minHeight, self.maxHeight,
		self.points_xyz)
		--]]
		self.nPoints = self.points_xyz:size(1)

	elseif self.location == 'chest' then
		
	    -- Print for debugging
		-- The raw readings are correct
		--[[
	    for i=1, X:size(1) do
	        print(string.format('Ranges: \t %.2f, \t%.2f, \t%.2f', X[i][1], X[i][2], X[i][3]))
	    end
		--]]
		
		
		self:update_chest( roll, pitch, yaw )
		
		------------------------------------------------
		-- Perform the transformations to the body frame
		self.points_xyz:resize( X:size(1), 4 )
		self.points_xyz:mm( X, self.T:t() )
		------------------------------------------------

	  -- Print for debugging
		-- The transformation is fine
		--[[
	    for i=1, X:size(1) do
	        print(string.format('Ranges: \t %.2f, \t%.2f, \t%.2f', self.points_xyz[i][1], self.points_xyz[i][2], self.points_xyz[i][3]))
	    end
		--]]
		
		
	else
		print('Wrong location of lidar!')
	end

end


local function update_head( self, roll, pitch, yaw )
--TODO: parameters need to be updated
	-----------------------
	-- Rotation of the lidar
	self.t_roll  = libTrig.rotx( roll )
	self.t_pitch = libTrig.roty( pitch )
	self.t_yaw   = libTrig.rotz( yaw )
	self.t_rotation = torch.mm(torch.mm(self.t_roll, self.t_pitch), self.t_yaw)
	-----------------------

	self.head_height = 0.15
	self.lidar_offestz = 0.10

	-----------------------
	-- Height off the chest to the head
	self.t_headOffset = libTrig.trans({0, 0, self.head_height})
	-----------------------

	-----------------------
	-- Transformation from the chest to the head
	self.t_body2head = torch.mm( self.t_headOffset, self.t_rotation )
	-----------------------

	-- Transformation from the holder to lidar
	self.t_lidarOffset = libTrig.trans( 
	{self.lidar_offsetx, self.lidar_offsety, self.lidar_offsetz}
	)

	-----------------------
	-- Transformation from the body to the lidar
	self.T = torch.mm(self.t_body2head, self.t_lidarOffset)
	-----------------------

end


local function update_chest( self, roll, pitch, yaw )
	-----------------------
	-- Rotation of the lidar
	self.t_roll  = libTrig.rotx( roll )
	self.t_pitch = libTrig.roty( pitch )
	self.t_yaw   = libTrig.rotz( yaw )
	self.t_rotation = torch.mm(torch.mm(self.t_roll, self.t_pitch), self.t_yaw)
	-----------------------

	-- Offset of lidar from chest (body center)
	self.lidar_offsetz = -0.05
	self.lidar_offsety = 0.05
	self.lidar_offsetx = 0.04

	-- Translation from the body to lidar
	self.t_translation = libTrig.trans( 
	{self.lidar_offsetx,self.lidar_offsety,self.lidar_offsetz}
	)

	-- Homogeneous transformaion from body to lidar
	self.T = torch.mm(torch.mm(self.t_rotation, self.t_translation), libTrig.rotx(-math.pi/2))
end



libLaser.initialize_lidar = 
function(lidar, location, minRange, maxRange, minHeight, maxHeight, minIdx, maxIdx)
	-- Location of the lidar: 'head' or 'chest'
	lidar.location = location

	-- Range limits
	lidar.minRange = minRange
	lidar.maxRange = maxRange

	-- Laser points height limits w.r.t body center
	lidar.minHeight = minHeight 
	lidar.maxHeight = maxHeight

	-- Make the point containers
	lidar.ranges = torch.FloatTensor( maxIdx-minIdx+1 ):zero()
	
	-- Relative XYZ points in the Hokuyo frame
	lidar.points = torch.Tensor(maxIdx-minIdx+1,4):zero()
	-- Fill the z and 4th (scale) coordinates
	lidar.points:select(2,3):fill(0)
	lidar.points:select(2,4):fill(1)
	-- Relative XYZ coordinates in the robot body frame
	lidar.points_xyz = torch.Tensor(maxIdx-minIdx+1,4):zero()
	
	
	-- Angle index limits
	lidar.minIdx = minIdx
	lidar.maxIdx = maxIdx
	lidar.FOV    = 270 * (maxIdx-minIdx+1) / 1081 -- This many degrees
	lidar.resd   = lidar.FOV / (maxIdx-minIdx) -- Kinda rounded, may cause size problem, need further check
	--lidar.res    = lidar.resd/180*math.pi
	local shift = ( minIdx-1 )/1081*270  -- degree

	-- Make the angles with proper start/end points. Hack the size of angles.
	if torch.range(0, lidar.FOV, lidar.resd):size(1) < lidar.ranges:size(1) then
		lidar.angles = torch.range(-135+shift-lidar.resd, -135+shift+lidar.FOV, lidar.resd)
	else
		lidar.angles = torch.range(-135+shift, -135+shift+lidar.FOV, lidar.resd)
	end
    --print(string.format('\n\n1st angle: %f, last angle: %f', lidar.angles[1], lidar.angles[-1]))
	
	-- Convert to radians and compute cos and sin
	local middleAngle = lidar.angles:mean()
	lidar.angles:mul(math.pi/180)
	lidar.cosines = torch.cos(lidar.angles)
	lidar.sines   = torch.sin(lidar.angles)
	
	-- Printing for debug
	--[[
	print(string.format(' Mid offset: %f', shift))
	print(string.format('Middle angle: %.3f',  middleAngle))
	print(string.format('1st angle: %f, last angle: %f \n\n', lidar.angles[1], lidar.angles[-1]))
	--]]
	

	-----------
	-- Methods
	-----------
	lidar.transform = transform
	lidar.update_head = update_head
	lidar.update_chest = update_chest

	return lidar
end


libLaser.initialize_tmp_variables = function(lidar)
	-- Number of points after pruning
	lidar.nRanges = 1081
	lidar.nPoints = 1081

	-- Temporary variables for transformation due to rotation
	lidar.t_roll = torch.Tensor(4,4):zero()
	lidar.t_pitch = torch.Tensor(4,4):zero()
	lidar.t_yaw = torch.Tensor(4,4):zero()
	lidar.t_rotation = torch.Tensor(4,4):zero()

	-- Temporay variables for transoframtion due to offset/translation
	lidar.head_height = 0
	lidar.t_headOffset = torch.Tensor(4,4):zero()
	lidar.t_body2head = torch.Tensor(4,4):zero()
	lidar.t_lidarOffset = torch.Tensor(4,4):zero()

	lidar.lidar_offsetx = 0
	lidar.lidar_offsety = 0
	lidar.lidar_offsetz = 0
	lidar.t_translation = torch.Tensor(4,4):zero()

	lidar.T = torch.Tensor(4,4):zero()
	return lidar
end

-- Generic lidar
libLaser.new_lidar = function(location, minRange, maxRange, minHeight, maxHeight, minIdx, maxIdx)
	local l = {}
	-- Initialization
	l = libLaser.initialize_lidar(l, location, minRange, maxRange, minHeight, maxHeight, minIdx, maxIdx)
	l = libLaser.initialize_tmp_variables( l )

	return l
end


-- Yield the library object
return libLaser
