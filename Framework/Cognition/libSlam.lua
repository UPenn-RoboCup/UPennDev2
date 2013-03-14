require('Slam');
require 'unix'
local Sensors = require 'sensors/Config_Sensors'
require 'torch'
torch.Tensor = torch.FloatTensor

local libSlam = {}

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
OMAP.res    = MAPS.res;
OMAP.invRes = MAPS.invRes;
OMAP.xmin   = MAPS.xmin;
OMAP.ymin   = MAPS.ymin;
OMAP.xmax   = MAPS.xmax;
OMAP.ymax   = MAPS.ymax;
OMAP.zmin   = MAPS.zmin;
OMAP.zmax   = MAPS.zmax;
OMAP.sizex  = MAPS.sizex;
OMAP.sizey  = MAPS.sizey;
OMAP.data = torch.ByteTensor(OMAP.sizex,OMAP.sizex)--:fill(127) -- uncertain
OMAP.timestamp = unix.time();
libSlam.OMAP = OMAP;
print('Map size:',libSlam.MAPS.sizex,libSlam.MAPS.sizey)

-- Setup SLAM values
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
local X = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
local Y = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
local W = torch.FloatTensor(4,Sensors.LIDAR0.nRays):fill(0)
--print("Contiguous",xs:isContiguous(),ys:isContiguous())

local scan_match_tune = {}
local pass1 = {};
-- Number of yaw positions to check
pass1.nyaw = 15;
pass1.dyaw = 1.0 * math.pi/180.0;
-- At this resolution
-- TODO: make this dependent on angular velocity / motion speed
--if abs(tLidar0-tEncoders) < 0.1
pass1.nxs  = 5;
pass1.nys  = 5;
-- resolution of the candidate poses
pass1.dx  = 0.02;
pass1.dy  = 0.02;
--else
--  nxs1  = 11;
--  nys1  = 11;
--  dx1   = 0.05;
--  dy1   = 0.05;
--end
-- create the candidate locations in each dimension
pass1.xCand = torch.range(-1*math.floor(pass1.nxs/2), math.floor(pass1.nxs/2) )
pass1.yCand = torch.range(-1*math.floor(pass1.nys/2), math.floor(pass1.nys/2) )
pass1.aCand = torch.range(-1*math.floor(pass1.nyaw/2),math.floor(pass1.nyaw/2))
-- TODO: This is the garage collected object, I believe!
pass1.hits = torch.DoubleTensor( 
pass1.xCand:nElement(), pass1.yCand:nElement(), pass1.aCand:nElement()
)
scan_match_tune[1] = pass1;

local pass2 = {};
pass2.nyaw = 21;
pass2.dyaw = 0.1 * math.pi/180.0;
pass2.nxs  = 1;
pass2.nys  = 1;
pass2.dx   = 0.01;
pass2.dy   = 0.01;
pass2.xCand = torch.range(-1*math.floor(pass2.nxs/2), math.floor(pass2.nxs/2) )
pass2.yCand = torch.range(-1*math.floor(pass2.nys/2), math.floor(pass2.nys/2) )
pass2.aCand = torch.range(-1*math.floor(pass2.nyaw/2),math.floor(pass2.nyaw/2))
pass2.hits = torch.DoubleTensor( 
pass2.xCand:nElement(), pass2.yCand:nElement(), pass2.aCand:nElement()
)
scan_match_tune[2] = pass2;

-- Process lidar readings as the come in
local function processL0()
  --print('processing lidar...')
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
    print('No good readings after initial checks.')
    return
  end

  -- Apply the transformation given current roll and pitch
  -- TODO: check if transpose...
  -- TODO: this is unstable!
--  T = torch.mm(
--  libSlam.roty(Sensors.IMU.data.P*math.pi/180),libSlam.rotx(Sensors.IMU.data.R*math.pi/180)
--  );
  T = torch.eye(4);
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
    print('No good readings after ground check.')
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
  if true then
    local hmax = libSlam.scanMatchOne();
    hmax = libSlam.scanMatchTwo();
    -- If no good fits, then use pure odometry readings
    if hmax < 500 then
print('no update!',hmax)
      SLAM.x = SLAM.xOdom;
      SLAM.y = SLAM.yOdom;
      SLAM.yaw = SLAM.yawOdom;
    end
    -- TODO: it is negative...?
--[[
SLAM.x = -1*SLAM.x;
SLAM.y = -1*SLAM.y;
--SLAM.yaw = -1*SLAM.yaw;
SLAM.yaw = Sensors.IMU.data.Y*math.pi/180
--]]
  else
    print('not moving');
  end

  -- Update the SLAM odometry
  SLAM.xOdom   = SLAM.x;
  SLAM.yOdom   = SLAM.y;
  SLAM.yawOdom = SLAM.yaw;

  ---------------------
  -- Update the map, since our pose was just updated
  ---------------------
  -- Take the laser scan points from the body frame and 
  -- put them in the world frame
  print('My x, y position is',SLAM.x, SLAM.y)
  local tmp = torch.mm( 
  libSlam.trans( {SLAM.x, SLAM.y, SLAM.z} ),
  libSlam.rotz( SLAM.yaw ) 
  )
  -- TODO: determine if transpose or not
  local T = torch.mm( 
  tmp, libSlam.trans( 
  {Sensors.LIDAR0.offsetx, Sensors.LIDAR0.offsety, Sensors.LIDAR0.offsetz}
  ))

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
  --OMAP.data:fill(0);
  for i=1,nranges do
    if xis[i]>1 and yis[i]>1 and xis[i]<OMAP.sizex and yis[i]<OMAP.sizey then
      OMAP.data[ xis[i] ][ yis[i] ] = OMAP.data[ xis[i] ][ yis[i] ] + inc;
      --OMAP.data[ xis[i] ][ yis[i] ] = 255;
      --print(OMAP.data[ xis[i] ][ yis[i] ] ,xis[i], yis[i] )
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

  -- The first pass sets up variables
  -- and does not attempt to match scans
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
    return 0;
  end

  -- Update the timestamps
  --tEncoders = ENCODERS.counts.t;
  tLidar0 = Sensors.LIDAR0.startTime;
  local wyaw = Sensors.IMU.data.Y - (Sensors.LIDAR0.last_imu_yaw or Sensors.IMU.data.Y);
  Sensors.LIDAR0.last_imu_yaw = Sensors.IMU.data.Y;
  -- TODO: it is negative
  wyaw = -1*wyaw*math.pi/180;
  --print('wyaw:',wyaw)

  -- Reset the ranges based on the current odometry
  -- TODO: determine how much to search over the yaw space based on 
  -- the instantaneous angular velocity from the imu
  local xCand = pass1.xCand;
  local yCand = pass1.yCand;
  local aCand = pass1.aCand;
  local hits = pass1.hits;
  xCand:range(-1*math.floor(pass1.nxs/2), math.floor(pass1.nxs/2) )
  yCand:range(-1*math.floor(pass1.nys/2), math.floor(pass1.nys/2) )
  aCand:range(-1*math.floor(pass1.nyaw/2),math.floor(pass1.nyaw/2))
  xCand:mul(pass1.dx):add(SLAM.xOdom);
  yCand:mul(pass1.dy):add(SLAM.yOdom);
  aCand:mul(pass1.dyaw):add(SLAM.yawOdom):add( wyaw );

  -- Zero the hits, which will be accumulated in ScanMatch2D
  hits:zero()
  -- TODO: Every other Y on the first pass
  local hmax, ixmax, iymax, ithmax = Slam.ScanMatch2D('match',
  OMAP.data,
  Y, -- Transformed points
  xCand, yCand, aCand
,hits
  );
--print( 'Best indices', hmax, xmax, ymax, thmax )
--print('Hit size',hits:size()[1],hits:size()[2],hits:size()[3])
--print('Best position:', xCand[ixmax], yCand[iymax], aCand[ithmax] )

  -- TODO: Create a better grid of distance-based costs
  -- from each cell to the odometry pose
  local minIndX, indx = torch.min( xCand:add(-SLAM.xOdom):abs(), 1 );
  local minIndY, indy = torch.min( yCand:add(-SLAM.yOdom):abs(), 1 );

  -- How valuable is the odometry preidiction?
  -- Should make a gaussian depression around this point...
  -- Extract the 2D slice of xy poses at the best angle to be the cost map

  local costGrid1 = hits:select(3,ithmax):mul(-1)
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
  SLAM.yaw = aCand[ithmax];
  SLAM.x   = xCand[mindex_x];
  SLAM.y   = yCand[mindex_y];
--SLAM.x, SLAM.y, SLAM.yaw = xCand[ixmax], yCand[iymax], aCand[ithmax]
  return hmax
end
libSlam.scanMatchOne = scanMatchOne

-----------------------------
-------- ScanMatchTwo -------
-----------------------------
local function scanMatchTwo()
  -- Reset the ranges based on the current odometry
  -- TODO: determine how much to search over the yaw space based on 
  -- the instantaneous angular velocity from the imu
  local xCand = pass2.xCand;
  local yCand = pass2.yCand;
  local aCand = pass2.aCand;
  local hits = pass2.hits;
  xCand:range(-1*math.floor(pass2.nxs/2), math.floor(pass2.nxs/2) )
  yCand:range(-1*math.floor(pass2.nys/2), math.floor(pass2.nys/2) )
  aCand:range(-1*math.floor(pass2.nyaw/2),math.floor(pass2.nyaw/2))
  xCand:mul(pass2.dx):add(SLAM.x);
  yCand:mul(pass2.dy):add(SLAM.y);
  aCand:mul(pass2.dyaw):add(SLAM.yaw);
  
  -- Zero the hits, which will be accumulated in ScanMatch2D
  hits:zero()

  -- TODO: Every other Y on the first pass
  local hmax, ixmax, iymax, ithmax = Slam.ScanMatch2D('match',
  OMAP.data,
  Y, -- Transformed points
  xCand, yCand, aCand
,hits
  );
  SLAM.yaw = aCand[ithmax];
  return hmax;
end
libSlam.scanMatchTwo = scanMatchTwo

-- Process the IMU data
local function processIMU( imu_tbl )
  --[[
  for i,v in pairs(imu_tbl) do
    print(i,v)
  end
  --]]
  Sensors.IMU.data = imu_tbl;
  --[[
  Sensors.IMU.dyaw = imu_tbl.data.Y - Sensors.IMU.lastY;
  Sensors.IMU.lastY = imu_tbl.data.Y;
  --]]
end
libSlam.processIMU = processIMU;

local function processOdometry()
end
libSlam.processOdometry = processOdometry;

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
