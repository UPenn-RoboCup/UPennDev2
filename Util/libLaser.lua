-- libLaser
-- (c) 2013 Stephen McGill
-- Filtering of the laser points from the device

require 'torch'
torch.Tensor = torch.DoubleTensor
local libTransform = require 'libTransform'
local Body = require'Body'
local K = Body.Kinematics

local libLaser = {}
-- Using a Hokuyo only for now
local READING_PER_DEGREE = 4
local READING_PER_RADIAN = Body.RAD_TO_DEG * READING_PER_DEGREE
local N_READINGS = 1081
local HALFWAY_READING = 541

local function transform(self, roll, pitch, yaw)
  --------------
  -- TODO: this resize should be unnecessary
  -- Reset the range container size
  self.ranges:resize( self.maxIdx - self.minIdx + 1 )
  self.points:resize( self.maxIdx - self.minIdx + 1, 4 )
  --------------

  --------------
  -- Easier access to data
  local rawX  = self.points
  local xs = rawX:select(2,1)
  local ys = rawX:select(2,2)
  --------------

  --------------
  -- Transform LIDAR readings into relative cartesian coordinate
  
  --print(string.format('\n\nSize of ranges: %d', self.ranges:size(1)))
  --print(string.format('Size of cosines: %d \n\n', self.cosines:size(1)))
  
  xs:copy(self.ranges):cmul( self.cosines )
  ys:copy(self.ranges):cmul( self.sines )
  --------------

  --------------
  ---[[ Prune based on the minimum and maximum ranges
  self.nRanges = slam.range_filter(
  self.ranges,
  self.minRange, self.maxRange,
  xs, ys
  )
  --]]

  print('\n Original # of ranges:', self.ranges:size(1))
  print('nRanges:', self.nRanges, '\n\n')
  
  if self.nRanges < 1 then
    return
  end
  --X:resize( self.nRanges, 4 )
  X = rawX:sub(1, self.nRanges)


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
  self.t_roll  = libTransform.rotX( roll )
  self.t_pitch = libTransform.rotY( pitch )
  self.t_yaw   = libTransform.rotZ( yaw )
  self.t_rotation = torch.mm(torch.mm(self.t_roll, self.t_pitch), self.t_yaw)
  -----------------------

  self.head_height = K.neckOffsetZ
  self.lidar_offestz = 0.10 --FIXME

  -----------------------
  -- Height off the chest to the head
  self.t_headOffset = libTransform.trans(0, 0, self.head_height)
  -----------------------

  -----------------------
  -- Transformation from the chest to the head
  self.t_body2head = torch.mm( self.t_headOffset, self.t_rotation )
  -----------------------

  -- Transformation from the holder to lidar
  self.t_lidarOffset = libTransform.trans( 
  self.lidar_offsetx, self.lidar_offsety, self.lidar_offsetz
  )

  -----------------------
  -- Transformation from the body to the lidar
  self.T = torch.mm(self.t_body2head, self.t_lidarOffset)
  -----------------------

end


local function update_chest( self, roll, pitch, yaw )
  -----------------------
  -- Rotation of the lidar
  self.t_roll  = libTransform.rotX( roll )
  self.t_pitch = libTransform.rotY( pitch )
  self.t_yaw   = libTransform.rotZ( yaw )
  self.t_rotation = torch.mm(torch.mm(self.t_roll, self.t_pitch), self.t_yaw)
  -----------------------

  -- Offset of lidar from chest (body center)
  -- FIXME
  self.lidar_offsetz = -0.05
  self.lidar_offsety = 0.05
  self.lidar_offsetx = 0.04

  -- Translation from the body to lidar
  self.t_translation = libTransform.trans( 
  self.lidar_offsetx,self.lidar_offsety,self.lidar_offsetz
  )

  -- Homogeneous transformaion from body to lidar
  self.T = torch.mm(torch.mm(self.t_rotation, self.t_translation), libTransform.rotX(-math.pi/2))
end


-- Make a new lidar
libLaser.new_lidar = 
function(location, minRange, maxRange, minHeight, maxHeight, minFOV, maxFOV)
  
  local lidar = {}

  -- Location of the lidar: 'head' or 'chest'
  lidar.location = location

  -- Range limits (-135 degrees to 135 degrees)
  lidar.minRange = minRange
  lidar.maxRange = maxRange

  -- Laser points height limits w.r.t body center
  lidar.minHeight = minHeight 
  lidar.maxHeight = maxHeight

  -- Angle index limits
  lidar.minIdx = minFOV * READING_PER_RADIAN + HALFWAY_READING
  lidar.maxIdx = maxFOV * READING_PER_RADIAN + HALFWAY_READING
  lidar.FOV    = maxFOV - minFOV
  lidar.N_POINTS = (lidar.maxIdx - lidar.minIdx)+1

  -- Container for the anlges
  lidar.angles = torch.range(minFOV,maxFOV,.25*Body.DEG_TO_RAD)
  -- Make the point containers
  lidar.ranges = torch.FloatTensor( lidar.N_POINTS ):zero()
  -- Relative XYZ points in the Hokuyo frame
  lidar.points = torch.Tensor(lidar.N_POINTS,4):zero()
  -- Fill the z and 4th (scale) coordinates
  lidar.points:select(2,3):fill(0)
  lidar.points:select(2,4):fill(1)
  -- Relative XYZ coordinates in the robot body frame
  lidar.points_xyz = torch.Tensor(lidar.N_POINTS,4):zero()
  
  -- Convert to radians and compute cos and sin
  local middleAngle = torch.mean( lidar.angles )
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

  --------
  -- Initialize vairables
  --------
  -- Number of points after pruning
  lidar.nRanges = 1081
  lidar.nPoints = 1081

  -- Temporary variables for transformation due to rotation
  lidar.t_roll     = torch.Tensor(4,4):zero()
  lidar.t_pitch    = torch.Tensor(4,4):zero()
  lidar.t_yaw      = torch.Tensor(4,4):zero()
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


-- Yield the library object
return libLaser
