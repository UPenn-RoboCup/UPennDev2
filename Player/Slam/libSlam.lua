local ffi = require 'ffi'
require('Slam');
local Sensors = require 'Config_Sensors'
require 'torch'
require 'tutil'
require 'ffi/torchffi'
torch.Tensor = torch.FloatTensor
require 'unix'

libSlam = {}

-- Default SLAM Map values
local MAPS = {}
MAPS.res        = .05;
MAPS.invRes     = 1/MAPS.res;
MAPS.windowSize = 10; -- meters to see 
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
SLAM.xOdom = 0
SLAM.yOdom = 0
SLAM.yawOdom = 0
SLAM.lidar0Cntr = 20;
libSlam.SLAM = SLAM;

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
	local ranges = Sensors.LIDAR0.ranges
  local nranges = Sensors.LIDAR0.nRays
  -- Put lidar readings into relative cartesian coordinate
  --print( 'Ranges sz:\t', ranges:size() )
  --print( 'Cosines sz:\t', LIDAR0.cosines:size() )
  xs = ranges:clone()
	xs:cmul( Sensors.LIDAR0.cosines )
  ys = ranges:clone()
	ys:cmul( Sensors.LIDAR0.sines )

  -- Accept only ranges that are sufficiently far away
  --dranges = [0; diff(ranges)];
  --  local indGood_s = torch.CharStorage(nranges):fill(0)
  --  local indGood = torch.Tensor(indGood_s);
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
  --print('Good returns:',nranges)
  xs:resize(nranges)
  ys:resize(nranges)

  -- Apply the transformation given current roll and pitch
  T = torch.mm(libSlam.roty(Sensors.IMU.pitch),libSlam.rotx(Sensors.IMU.roll)):t();
  --T = torch.eye(4);
  --X = [xsg ysg zsg onesg];
  -- TODO: for memory efficiency, this Tensor should be 
  -- declared up top, and "views" should just be manipulated as
  -- xs, ys, zs.  Look at the memory allocation, so it's 
  -- just [<---xs---><---ys---><---zs--->]
  X = torch.Tensor(nranges,4)
  X:select(2,1):copy(xs)
  X:select(2,2):copy(ys)
  X:select(2,3):fill(0)
  X:select(2,4):fill(1);
  Y=torch.mm(X,T);  --reverse the order because of transpose
	-- TODO: Don't make Y as a new memory place - do inplace 
	-- multiply and store to X

  -- Do not use the points that supposedly hit the ground (check!!)
  -- TODO: Make fast masking!
  xs = Y:select(2,1);
  ys = Y:select(2,2);
  zs = Y:select(2,3);
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
  -- print('Good returns above floor:',nranges)
	
	-- TODO: This seems wrong!
  Y:resize(nranges,4)
  -- Reset the views
  xs = Y:select(2,1);
  ys = Y:select(2,2);
  zs = Y:select(2,3);
  --gnuplot.figure(2)
  --gnuplot.plot('Ranges',xs,ys,'+')

  -- These are now the default cartesian points
  --LIDAR0.xs = xs;
  --LIDAR0.ys = ys;

  ----------------------
  -- Begin the scan matching to update the SLAM pose
  ----------------------
  -- if encoders are zero, don't move
  -- if(SLAM.odomChanged > 0)
  if true then

    -- figure out how much to search over the yaw space based on the 
    -- instantaneous angular velocity from imu

    -- Perform the scan matching
    -- TODO
		libSlam.scanMatchOne( xs, ys );
--    slamScanMatchPass2();

    -- If no good fits, then use pure odometry readings
    local hmax = 1000;
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
  SLAM.xOdom      = SLAM.x;
  SLAM.yOdom      = SLAM.y;
  SLAM.yawOdom    = SLAM.yaw;
  --]]
  -- Log some data
  --[[
  if (POSES.log)
  POSES.cntr = POSES.cntr + 1;
  POSES.data(:,POSES.cntr) = [SLAM.x; SLAM.y; SLAM.z; IMU.data.roll; IMU.data.pitch; SLAM.yaw];
  POSES.ts(POSES.cntr) = LIDAR0.scan.startTime;
  end
  --]]

  --
  -- Update the map, since our pose was just updated
  --

  -- Take the laser scan points from the body frame and put them in the
  -- world frame

  -- TODO
  --SLAM = {}
  SLAM.x = 0;
  SLAM.y = 0;
  SLAM.z = 0;
  SLAM.yaw = 0;
  local tmp = torch.mm( 
    libSlam.trans( {SLAM.x, SLAM.y, SLAM.z} ),
    libSlam.rotz(SLAM.yaw) 
    )
  T = torch.mm( 
    tmp, 
    libSlam.trans( {Sensors.LIDAR0.offsetx, Sensors.LIDAR0.offsety, Sensors.LIDAR0.offsetz}) 
  ):t()
  
  -- X = [xsss ysss zsss onez];
  -- TODO: for memory efficiency, this Tensor should be 
  -- declared up top, and "views" should just be manipulated as
  -- xs, ys, zs.  Look at the memory allocation, so it's 
  -- just [<---xs---><---ys---><---zs--->]
  X = torch.Tensor(nranges,4)
  X:select(2,1):copy(xs)
  X:select(2,2):copy(ys)
  X:select(2,3):copy(zs)
  X:select(2,4):fill(1);
  Y=torch.mm(X,T);  --reverse the order because of transpose

  -- Separate cartesian coordinates
  xs = Y:select(2,1);
  ys = Y:select(2,2);

  -- Convert each cartesian point to a map index
  xis = (xs - OMAP.xmin) * OMAP.invRes;
  yis = (ys - OMAP.ymin) * OMAP.invRes;
  xis:ceil()
  yis:ceil()

  -- By how much should should we update the map
  -- at each index from a laser return?
  local inc = 5;
  -- TODO
  --[[
  if SLAMMER.lidar0Cntr == 1 then
    inc=100;
  end
  --]]
--  gnuplot.figure(3)
--  gnuplot.imagesc( OMAP.data,'color')
  for i=1,nranges do
    --print('Evaluating ',xis[i],yis[i])
    if xis[i]>1 and yis[i]>1 and xis[i]<OMAP.sizex and yis[i]<OMAP.sizey then
      OMAP.data[ xis[i] ][ yis[i] ] = OMAP.data[ xis[i] ][ yis[i] ] + inc;
    end
  end
	OMAP.timestamp = unix.time();
  --gnuplot.figure(4)
  --gnuplot.imagesc( OMAP.data,'color')

  -- TODO
  -- Decay the map around the robot
  
  if SLAM.lidar0Cntr%20 == 0 then
    -- Get the map indicies for the robot
    xiCenter = math.ceil((SLAM.x - OMAP.xmin) * OMAP.invRes);
    yiCenter = math.ceil((SLAM.y - OMAP.ymin) * OMAP.invRes);

    -- Amount of the surrounding to decay
    --windowSize = 30 *OMAP.invRes;
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
    --
    -- merge the small map back into the full map
    -- TODO

    OMAP.data:sub( ximin,ximax,   yimin,yimax ):copy( localMap );
		OMAP.timestamp = unix.time();
  end
  -- Finished the decay



  --
  -- Determine if we need to shift the map
  --

  -- Always shift by a standard amount
  shiftAmount = 10; --meters
  xShift = 0;
  yShift = 0;
  -- Check in which directions we need to shift
  if (SLAM.x - OMAP.xmin < MAPS.edgeProx) then xShift = -shiftAmount; end
  if (SLAM.y - OMAP.ymin < MAPS.edgeProx) then yShift = -shiftAmount; end
  if (OMAP.xmax - SLAM.x < MAPS.edgeProx) then xShift = shiftAmount; end
  if (OMAP.ymax - SLAM.y < MAPS.edgeProx) then yShift = shiftAmount; end

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

local function scanMatchOne( xs, ys )
  --tEncoders = ENCODERS.counts.t;
  tLidar0   = Sensors.LIDAR0.startTime;
	xCand1:range(-xRange1,xRange1):mul(dx1):add(SLAM.xOdom);
	yCand1:range(-yRange1,yRange1):mul(dy1):add(SLAM.yOdom);
	aCand1:range(-yawRange1,yawRange1):mul(dyaw1):add(SLAM.yawOdom); -- + IMU.data.wyaw*0.025;
  hits:zero()
		
  local hmax, xmax, ymax, thmax = Slam.ScanMatch2D('match',
  OMAP.data,
  xs, ys,
  xCand1,yCand1,aCand1,
  hits
  );

  -- TODO: unfold to mean the 1:2:end syntax?
  -- NOTE: xs should be xsss(1:2:end) (same for ys)

  if (SLAM.lidar0Cntr > 1) then

    -- Create a grid of distance-based costs from each cell to odometry pose
    --yGrid1, xGrid1 = meshgrid( yCand1, xCand1 );
    --xDiff1 = xGrid1 - SLAM.xOdom;
    --yDiff1 = yGrid1 - SLAM.yOdom;
    --distGrid1 = (1/3)*1e6*torch.sqrt(xDiff1:pow(2) + yDiff1:pow(2) );
    minIndX, indx = torch.min( xCand1:add(-SLAM.xOdom):abs(), 1 );
    minIndY, indy = torch.min( yCand1:add(-SLAM.yOdom):abs(), 1 );
		
--print( indx[1], minIndX[1], indy[1], minIndY[1] )

    -- How valuable is the odometry preidiction?
    -- Should make a gaussian depression around this point...
    -- Extract the 2D slice of xy poses at the best angle to be the cost map
    local costGrid1 = hits:select(3,thmax):mul(-1)
--	print( "costGrid:",costGrid1:nDimension(), costGrid1:size()[1], costGrid1:size()[2]  )
--	print(indx,indx)
		
    costGrid1[indx[1]][indy[1]] = costGrid1[indx[1]][indy[1]] - 500; --  - 2e4;
    -- Find the minimum and save the new pose
    cmin, cimin = torch.min( costGrid1:resize( costGrid1:nElement() ) );

    -- Save the best pose
    SLAM.yaw = aCand1[thmax];
    SLAM.x   = xCand1[cimin];
    SLAM.y   = yCand1[cimin];
  else
    xStart, yStart, thStart = FindStartPose(SLAM.x, SLAM.y, SLAM.yaw, xsss,ysss);

    --if not isempty(xStart) then
    if xStart then
      SLAM.x = xStart;
      SLAM.Y = yStart;
      SLAM.yaw = thStart;
      SLAM.xOdom = xStart;
      SLAM.yOdom = yStart;
      SLAM.yawOdom = thStart;
    end

  end
end
libSlam.scanMatchOne = scanMatchOne

-- mapShift(myMap,x,y)
-- myMap: Map table to be copied.
-- x: Amount (in meters) to shift map by in the x direction
-- y: Amount (in meters) to shift map by in the y direction
-- Steve McGill's Torch copy of Alex's mapResize function
	--mapShift( OMAP, 5*OMAP.res, 5*OMAP.res )
	
function libSlam.mapShift(myMap,x,y)

	-- If no shifting is necessary, then just exit!
if x==0 and y==0 then
  return;
end

-- Make a memory space for the new map
newMapData = myMap.data:clone():zero();

-- Convert meters to map indices
dxi = torch.ceil(torch.abs(x)*myMap.invRes);
dyi = torch.ceil(torch.abs(y)*myMap.invRes);

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
  t[1][4] = v[1];
  t[2][4] = v[2];
  t[3][4] = v[3];
  return t;
end
libSlam.trans = trans

return libSlam