--
-- Lidar0 message handler (horizontal lidar)
--

require('Slam');
print()
for k,v in pairs(Slam) do
  print(k,v);
end
print('\n=== Processing LIDAR0 ===\n')
require 'scanMatchOne'

function processL0( LIDAR0, IMU, OMAP, MAPS )

  local ranges = LIDAR0.ranges;
  local nranges = LIDAR0.nRays;
  -- Put lidar readings into relative cartesian coordinate
  --print( 'Ranges sz:\t', ranges:size() )
  --print( 'Cosines sz:\t', LIDAR0.cosines:size() )
  xs = ranges:clone():cmul( LIDAR0.cosines );
  ys = ranges:clone():cmul( LIDAR0.sines );

  -- Accept only ranges that are sufficiently far away
  --dranges = [0; diff(ranges)];
  --  local indGood_s = torch.CharStorage(nranges):fill(0)
  --  local indGood = torch.Tensor(indGood_s);
  -- TODO: Make fast masking!
  local good_cnt = 0;
  for i=1,nranges do
    --indGood[i] = ranges[i]>0.25 and LIDAR0.mask and abs(dranges)<0.1;
    if ranges[i]>0.25 and LIDAR0.mask then
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
  T = torch.mm(roty(IMU.pitch),rotx(IMU.roll)):t();
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
  LIDAR0.xs = xs;
  LIDAR0.ys = ys;

  ----------------------
  -- Begin the scan matching to update the SLAM pose
  ----------------------
  -- if encoders are zero, don't move
  -- if(SLAM.odomChanged > 0)
  if true then

    -- figure out how much to search over the yaw space based on the 
    -- instantaneous angular velocity from imu

    -- Perfrom the scan matching
    -- TODO
    scanMatchOne( LIDAR0, OMAP, xs, ys );
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
  SLAM = {}
  SLAM.x = 0;
  SLAM.y = 0;
  SLAM.z = 0;
  SLAM.yaw = 0;
  local tmp = torch.mm( 
    trans( {SLAM.x, SLAM.y, SLAM.z} ),
    rotz(SLAM.yaw) 
    )
  T = torch.mm( 
    tmp, 
    trans( {LIDAR0.offsetx, LIDAR0.offsety, LIDAR0.offsetz}) 
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
  if SLAM.lidar0Cntr == 1 then
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
  --gnuplot.figure(4)
  --gnuplot.imagesc( OMAP.data,'color')

  -- TODO
  -- Decay the map around the robot
  --[[
  if SLAM.lidar0Cntr%20 == 0 then
    -- Get the map indicies for the robot
    xiCenter = ceil((SLAM.x - OMAP.xmin) * OMAP.invRes);
    yiCenter = ceil((SLAM.y - OMAP.ymin) * OMAP.invRes);

    -- Amount of the surrounding to decay
    --windowSize = 30 *OMAP.invRes;
    windowSize = 10 * OMAP.invRes;
    -- Get the surrounding's extreme indicies
    ximin = ceil(xiCenter - windowSize/2);
    ximax = ximin + windowSize - 1;
    yimin = ceil(yiCenter - windowSize/2);
    yimax = yimin + windowSize - 1;

    -- Clamp if the surrounding exceeds the map boundaries
    if ximin < 1 then
      ximin = 1;
    end
    if ximax > OMAP.map.sizex then
      ximax = OMAP.map.sizex;
    end
    if yimin < 1 then
      yimin=1;
    end
    if yimax > OMAP.map.sizey then
      yimax= OMAP.map.sizey;
    end

    -- Perform the decay on the surreoundings
    localMap = OMAP.data:sub( ximin,ximax,  yimin,yimax );
    -- TODO
    --indd = localMap<50 & localMap > 0;
    --localMap(indd) = localMap(indd)*0.95;
    --localMap(localMap>100) = 100;
    --
    -- merge the small map back into the full map
    -- TODO
    --OMAP.data:sub( ximin,ximax,   yimin,yimax ) = localMap;

  end
  -- Finished the decay
--]]


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

-- TODO: Make sure the helper functions are working properly!
function rotx(t)
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


function roty(t)
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

function rotz(t)
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

function trans(v)
  local t = torch.eye(4);
  t[1][4] = v[1];
  t[2][4] = v[2];
  t[3][4] = v[3];
  return t;
end
