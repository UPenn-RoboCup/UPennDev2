local ffi = require 'ffi'
require('Slam');
local Sensors = require 'sensors/Config_Sensors'
require 'torch'
require 'ffi/torchffi'
torch.Tensor = torch.FloatTensor
require 'unix'

libSlam = {}

-- Default SLAM Map values
local MAPS = {}
--MAPS.res        = .1;
MAPS.res        = .05;
MAPS.invRes     = 1/MAPS.res;
--MAPS.windowSize = 10; -- meters to see 
MAPS.windowSize = 15; -- meters to see 
MAPS.edgeProx   = 2;
MAPS.xmin       = 0 - MAPS.windowSize;
MAPS.ymin       = 0 - MAPS.windowSize;
MAPS.xmax       = 0 + MAPS.windowSize;
MAPS.ymax       = 0 + MAPS.windowSize;
MAPS.zmin       = 0;
MAPS.zmax       = 5;
MAPS.sizex  = (MAPS.xmax - MAPS.xmin) / MAPS.res + 1;
MAPS.sizey  = (MAPS.ymax - MAPS.ymin) / MAPS.res + 1;
libSlam.MAPS = MAPS;

-- Occupancy Map
local OMAP = {}
OMAP.res        = MAPS.res;
OMAP.invRes     = MAPS.invRes;
OMAP.xmin       = MAPS.xmin;
OMAP.ymin       = MAPS.ymin;
OMAP.xmax       = MAPS.xmax;
OMAP.ymax       = MAPS.ymax;
OMAP.zmin       = MAPS.zmin;
OMAP.zmax       = MAPS.zmax;
OMAP.sizex  = MAPS.sizex;
OMAP.sizey  = MAPS.sizey;
--OMAP.data = torch.Tensor(OMAP.sizex,OMAP.sizex):zero()
OMAP.data = torch.ByteTensor(OMAP.sizex,OMAP.sizex)
OMAP.timestamp = unix.time();
libSlam.OMAP = OMAP;

-- Setup SLAM thing
local SLAM = {}
SLAM.x = 0;
SLAM.y = 0;
SLAM.z = 0;
SLAM.yaw = 0;
SLAM.xOdom = 0
SLAM.yOdom = 0
SLAM.yawOdom = 0
SLAM.lidar0Cntr = 0;
libSlam.SLAM = SLAM;

-- Set the resolution for our C program
Slam.ScanMatch2D( 'setResolution', MAPS.res );
Slam.ScanMatch2D( 'setBoundaries', OMAP.xmin, OMAP.ymin, OMAP.xmax, OMAP.ymax );
Slam.ScanMatch2D( 'setSensorOffsets', 
Sensors.LIDAR0.offsetx, Sensors.LIDAR0.offsety, Sensors.LIDAR0.offsetz
);

-- Instantiate to be big enough if all lidar hits are considered
-- These tensors hold (x,y,z,t) points for each lidar point
-- Each undergoes some different type of transformation
X = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
Y = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
W = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
--print("Contiguous",xs:isContiguous(),ys:isContiguous())

-- Number of yaw positions to check
nyaw1 = 15;
dyaw1 = 1.0 * math.pi/180.0;
-- At this resolution
-- TODO: make this dependent on angular velocity / motion speed
--if abs(tLidar0-tEncoders) < 0.1
nxs1  = 5;
nys1  = 5;
-- resolution of the candidate poses
dx1   = 0.02;
dy1   = 0.02;
--else
--  nxs1  = 11;
--  nys1  = 11;
--  dx1   = 0.05;
--  dy1   = 0.05;
--end

yawRange1 = math.floor(nyaw1/2);
xRange1   = math.floor(nxs1/2);
yRange1   = math.floor(nys1/2);

-- create the candidate locations in each dimension
xCand1 = torch.range(-xRange1,xRange1)
yCand1 = torch.range(-yRange1,yRange1)
aCand1 = torch.range(-yawRange1,yawRange1)
hits = torch.DoubleTensor( 
xCand1:nElement(), yCand1:nElement(), aCand1:nElement()
)

-- Process lidar readings as the come in
local function processL0()
  -- Easier to read accessors
  local ranges = Sensors.LIDAR0.ranges
  local nranges = Sensors.LIDAR0.nRays

  -- Increment the lidar counter
  SLAM.lidar0Cntr = SLAM.lidar0Cntr + 1

  -- Put lidar readings into relative cartesian coordinate
  X:resize(4,nranges)
  xs = X:select(1,1);
  ys = X:select(1,2);
  xs:copy(ranges):cmul( Sensors.LIDAR0.cosines )
  ys:copy(ranges):cmul( Sensors.LIDAR0.sines )

  -- Accept only ranges that are sufficiently far away
  -- TODO: Make fast masking!
  local good_cnt = 0;
  for i=1,nranges do
    --indGood[i] = ranges[i]>0.25 and LIDAR0.mask and abs(dranges)<0.1;
    if ranges[i]>0.25 and Sensors.LIDAR0.mask then
      good_cnt = good_cnt+1;
      xs[good_cnt] = xs[i];
      ys[good_cnt] = ys[i];
    end
  end
  -- Resize to include just the good readings
  nranges = good_cnt;
  if nranges==0 then
    return
  end

  -- Apply the transformation given current roll and pitch
  -- TODO: check if transpose...
  T = torch.mm(libSlam.roty(Sensors.IMU.pitch),libSlam.rotx(Sensors.IMU.roll));
  --T = torch.eye(4);
  X:resize(4,nranges)
  Y:resize(4,nranges)
  X:select(1,3):fill(0); -- z
  X:select(1,4):fill(1); -- extra
  Y:mm(T,X);  --reverse the order because of transpose
  xs = Y:select(1,1);
  ys = Y:select(1,2);
  zs = Y:select(1,3);

  -- Do not use the points that supposedly hit the ground (check!!)
  -- TODO: Make fast masking!
  local good_cnt = 0;
  for i=1,nranges do
    if zs[i]>-0.3 then
      good_cnt = good_cnt+1;
      xs[good_cnt] = xs[i];
      ys[good_cnt] = ys[i];
      zs[good_cnt] = zs[i];
    end
  end
  nranges = good_cnt;
  if nranges==0 then
    return
  end

  -- Reset the views
  X:resize(4,nranges)
  Y:resize(4,nranges)
  xs = Y:select(1,1);
  ys = Y:select(1,2);
  zs = Y:select(1,3);

  ----------------------
  -- Begin the scan matching to update the SLAM pose
  ----------------------
  -- if encoders are zero, don't move
  -- if(SLAM.odomChanged > 0)
  if false then
    local hmax = libSlam.scanMatchOne();
    print( "hmax", hmax )
    -- TODO
    -- slamScanMatchPass2();

    -- If no good fits, then use pure odometry readings
    -- hmax = 1000;
    if hmax < 500 then
      SLAM.x = SLAM.xOdom;
      SLAM.y = SLAM.yOdom;
      SLAM.yaw = SLAM.yawOdom;
    end

  else
    print('not moving');
  end

  -- Save pose data from SLAM to the 
  -- global POSE structure
  -- TODO
  --[[
  POSE.data.x     = SLAM.x;
  POSE.data.y     = SLAM.y;
  POSE.data.z     = SLAM.z;
  POSE.data.roll  = IMU.data.roll;
  POSE.data.pitch = IMU.data.pitch;
  POSE.data.yaw   = SLAM.yaw;
  POSE.data.t     = unix.time();
  POSE.t          = unix.time();
	--]]
  SLAM.xOdom      = SLAM.x;
  SLAM.yOdom      = SLAM.y;
  SLAM.yawOdom    = SLAM.yaw;

  ---------------------
  -- Update the map, since our pose was just updated
  ---------------------
  -- Take the laser scan points from the body frame and 
  -- put them in the world frame
  print('Current Slam:',SLAM.x, SLAM.y, SLAM.z, SLAM.yaw)
  local tmp = torch.mm( 
  libSlam.trans( {SLAM.x, SLAM.y, SLAM.z} ),
  libSlam.rotz( SLAM.yaw ) 
  )
  -- TODO: determine if transpose or not
  local T = torch.mm( 
  tmp, 
  libSlam.trans( {Sensors.LIDAR0.offsetx, Sensors.LIDAR0.offsety, Sensors.LIDAR0.offsetz}) 
  )

  -- Put the points into the slam frame
  W:resize(4,nranges)
  W = torch.mm(T,Y);
  xs = W:select(1,1);
  ys = W:select(1,2);

  -- Convert each cartesian point to a map index
  xis = (xs - OMAP.xmin) * OMAP.invRes;
  yis = (ys - OMAP.ymin) * OMAP.invRes;
  xis:ceil()
  yis:ceil()

  -- By how much should should we update the map
  -- at each index from a laser return?
  local inc = 5;
  if SLAM.lidar0Cntr == 1 then
    inc=100;
  end

  ---------------
  -- Perform the map update
  ---------------
  for i=1,nranges do
    if xis[i]>1 and yis[i]>1 and xis[i]<OMAP.sizex and yis[i]<OMAP.sizey then
      OMAP.data[ xis[i] ][ yis[i] ] = OMAP.data[ xis[i] ][ yis[i] ] + inc;
    end
  end
  OMAP.timestamp = unix.time();

  ---------------
  -- Decay the map around the robot
  ---------------
  if SLAM.lidar0Cntr%20 == 0 then
    -- Get the map indicies for the robot
    xiCenter = math.ceil((SLAM.x - OMAP.xmin) * OMAP.invRes);
    yiCenter = math.ceil((SLAM.y - OMAP.ymin) * OMAP.invRes);

    -- Amount of the surrounding to decay
    --windowSize = 30 * OMAP.invRes;
    windowSize = 10 * OMAP.invRes;
    -- Get the surrounding's extreme indicies
    ximin = math.ceil(xiCenter - windowSize/2);
    ximax = ximin + windowSize - 1;
    yimin = math.ceil(yiCenter - windowSize/2);
    yimax = yimin + windowSize - 1;

    -- Clamp if the surrounding exceeds the map boundaries
    if ximin < 1 then
      ximin = 1;
    end
    if ximax > OMAP.sizex then
      ximax = OMAP.sizex;
    end
    if yimin < 1 then
      yimin=1;
    end
    if yimax > OMAP.sizey then
      yimax= OMAP.sizey;
    end

    -- Perform the decay on the surroundings
    localMap = OMAP.data:sub( ximin,ximax,  yimin,yimax );
    -- TODO
    --indd = localMap<50 & localMap > 0;
    --localMap(indd) = localMap(indd)*0.95;
    --localMap(localMap>100) = 100;

    -- Merge the small map back into the full map
    OMAP.data:sub( ximin,ximax,   yimin,yimax ):copy( localMap );
    OMAP.timestamp = unix.time();
  end
  -- Finished the decay

  ------------
  -- Determine if we need to shift the map
  ------------
  shiftAmount = 10; -- Always shift by a standard amount (in meters)
  xShift = 0;
  yShift = 0;
  -- Check in which directions we need to shift
  if (SLAM.x - OMAP.xmin < MAPS.edgeProx) then xShift = -shiftAmount
  elseif (OMAP.xmax - SLAM.x < MAPS.edgeProx) then xShift = shiftAmount
  end
  if (SLAM.y - OMAP.ymin < MAPS.edgeProx) then yShift = -shiftAmount
  elseif (OMAP.ymax - SLAM.y < MAPS.edgeProx) then yShift = shiftAmount;
  end
  -- Perform the shift via helper functions
  if xShift ~= 0 or yShift ~= 0 then
    mapShift(OMAP,xShift,yShift);
    Slam.ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
  end

  -- TODO
  -- Set the last updated time
  --LIDAR0.lastTime = LIDAR0.scan.startTime;
end
libSlam.processL0 = processL0

local function scanMatchOne()
  --tEncoders = ENCODERS.counts.t;
  tLidar0 = Sensors.LIDAR0.startTime;
  xCand1:range(-xRange1,xRange1):mul(dx1):add(SLAM.xOdom);
  yCand1:range(-yRange1,yRange1):mul(dy1):add(SLAM.yOdom);
  -- TODO: determine how much to search over the yaw space based on 
  -- the instantaneous angular velocity from the imu
  aCand1:range(-yawRange1,yawRange1):mul(dyaw1):add(SLAM.yawOdom); -- + IMU.data.wyaw*0.025;
  hits:zero()
  local hmax, xmax, ymax, thmax = Slam.ScanMatch2D('match',
  OMAP.data,
  Y, -- Transformed points
  xCand1,yCand1,aCand1,
  hits
  );

  -- Is this our first pass?
  if SLAM.lidar0Cntr <= 1 then
    -- TODO: add this function
    --xStart, yStart, thStart = FindStartPose(SLAM.x, SLAM.y, SLAM.yaw, xsss,ysss);
    print('First LIDAR pass!')
    if xStart then
      SLAM.x = xStart;
      SLAM.Y = yStart;
      SLAM.yaw = thStart;
      SLAM.xOdom = xStart;
      SLAM.yOdom = yStart;
      SLAM.yawOdom = thStart;
    end

  else
    -- TODO: Create a better grid of distance-based costs
    -- from each cell to the odometry pose
    local minIndX, indx = torch.min( xCand1:add(-SLAM.xOdom):abs(), 1 );
    local minIndY, indy = torch.min( yCand1:add(-SLAM.yOdom):abs(), 1 );

    -- How valuable is the odometry preidiction?
    -- Should make a gaussian depression around this point...
    -- Extract the 2D slice of xy poses at the best angle to be the cost map
    local costGrid1 = hits:select(3,thmax):mul(-1)
    --print('2d slice:',costGrid1:size()[1],costGrid1:size()[2])
    costGrid1[indx[1]][indy[1]] = costGrid1[indx[1]][indy[1]] - 500; --  - 2e4;
    -- Find the minimum and save the new pose
    local min_cost = costGrid1[1][1];
    local mindex_x = 1;
    local mindex_y = 1;
    for ii=1,costGrid1:size()[1] do
      for jj=1,costGrid1:size()[2] do
        if costGrid1[ii][jj]<min_cost then
          mindex_x = ii;
          mindex_x = jj;
          min_cost = costGrid1[ii][jj];
        end
      end
    end
    -- Save the best pose
    SLAM.yaw = aCand1[thmax];
    SLAM.x   = xCand1[mindex_x];
    SLAM.y   = yCand1[mindex_y];
  end
  return hmax
end
libSlam.scanMatchOne = scanMatchOne

-- mapShift(myMap,x,y)
-- myMap: Map table to be copied.
-- x: Amount (in meters) to shift map by in the x direction
-- y: Amount (in meters) to shift map by in the y direction
-- Use: mapShift( OMAP, 5*OMAP.res, 5*OMAP.res )
local function mapShift(myMap,x,y)
  -- If no shifting is necessary, then just exit!
  if x==0 and y==0 then
    return;
  end
  -- Make a memory space for the new map
  local newMapData = myMap.data:clone():zero();
  -- Convert meters to map indices
  local dxi = torch.ceil(torch.abs(x)*myMap.invRes);
  local dyi = torch.ceil(torch.abs(y)*myMap.invRes);

  -- Find the indices of the map data that we wish to preserve
  -- Find the new indicies to place this data into on the new map
  if x>0 then
    xis  = torch.range( 1+dxi, myMap.sizex );
    nxis = torch.range( 1, myMap.sizex-dxi );
  elseif x<0 then
    xis  = torch.range( 1, myMap.sizex-dxi );
    nxis = torch.range( 1+dxi, myMap.sizex );
  else
    xis  = torch.range( 1, myMap.sizex );
    nxis = torch.range( 1, myMap.sizex);
  end
  if y>0 then
    yis  = torch.range( 1+dyi, myMap.sizey );
    nyis = torch.range( 1, myMap.sizey-dyi );
  elseif y<0 then
    yis  = torch.range( 1, myMap.sizey-dyi );
    nyis = torch.range( 1+dyi, myMap.sizey );
  else
    yis  = torch.range( 1, myMap.map.sizey );
    nyis = torch.range( 1, myMap.map.sizey );
  end

  -- View the just the map which we wish to preserve
  keepMap = myMap.data:sub( xis[1],xis[-1],  yis[1],yis[-1] );
  -- View the just the new map which we wish to place the preserved data
  local newKeep = newMapData:sub( nxis[1],nxis[-1], nyis[1],nyis[-1] );
  -- Perform the copy from old map to new map
  newKeep:copy(keepMap)

  -- Update the map information based on the shift
  myMap.xmin = myMap.xmin + x;
  myMap.ymin = myMap.ymin + y;
  myMap.xmax = myMap.xmax + x;
  myMap.ymax = myMap.ymax + y;

  -- Discard the old map by reassigning new map
  -- NOTE: Garbage collection should release this memory
  myMap.data = newMapData;

  print( string.format(
  'Map (%s) shifted. New bounding box: x(%f %f) y(%f %f)\n',
  myMap.name,myMap.xmin,myMap.xmax,myMap.ymin,myMap.ymax
  ));

end
libSlam.mapShift = mapShift

-- TODO: Make sure the helper functions are working properly!
local function rotx(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the X axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[2][2] = ct;
  r[3][3] = ct;
  r[2][3] = -1*st;
  r[3][2] = st;
  return r
end
libSlam.rotx = rotx

local function roty(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Y axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[3][3] = ct;
  r[1][3] = st;
  r[3][1] = -1*st;
  return r
end
libSlam.roty = roty

local function rotz(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Y axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[2][2] = ct;
  r[1][2] = -1*st;
  r[2][1] = st;
  return r
end
libSlam.rotz = rotz

local function trans(v)
  local t = torch.eye(4);
  t[1][4] = v[1] or 0;
  t[2][4] = v[2] or 0;
  t[3][4] = v[3] or 0;
  return t;
end
libSlam.trans = trans

return libSlam
