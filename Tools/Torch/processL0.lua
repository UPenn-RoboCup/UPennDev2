--
-- Lidar0 message handler (horizontal lidar)
--

function processL0(data,name)

  -- If the data is not nil, then make it globally available
  if data then
    LIDAR0.scan = data;
  else
    return;
  end

  -- If this is not our first processing
  if LIDAR0_TS.cntr > 1 then
    tnow = unix.time();
    tl   = LIDAR0.scan.startTime;
    dtt= tnow-tl;
    tmod = mod(LIDAR0_TS.cntr-1,1000)+1;
    LIDAR0_TS.ts(tmod) = tl;
    LIDAR0_TS.dts(tmod) = dtt; --tl - LIDAR0_TS.ts(LIDAR0_TS.cntr-1);%dtt;
  end
  LIDAR0_TS.cntr = LIDAR0_TS.cntr + 1;

  -- Check that our IMU data is valid
  if not CheckImu() then
    disp('ignoring lidar0 because imu data is invalid');
    return;
  end

  -- Update the processing count
  SLAM.lidar0Cntr = SLAM.lidar0Cntr+1;
  if isempty(LIDAR0.lastTime)
    LIDAR0.lastTime = LIDAR0.scan.startTime;
  end

  -- Print some debugging messages
  if SLAM.lidar0Cntr%40 == 0 then
    fprintf(1,'.');
  end

  -- Convert from float to double, since lidar readings are floats
  ranges = double(LIDAR0.scan.ranges);
  -- Put lidar readings into relative cartesian coordinate
  xs = ranges.*LIDAR0.cosines;
  ys = ranges.*LIDAR0.sines;
  zs = zeros(size(xs));

  -- Accept only ranges that are sufficiently far away
  --dranges = [0; diff(ranges)];
  --indGood = ranges>0.25 & LIDAR0.mask & (abs(dranges) <0.1);
  indGood = ranges>0.25 & LIDAR0.mask;
  -- Select only the "good" readings
  xsg=xs(indGood);
  ysg=ys(indGood);
  zsg=zs(indGood);
  onesg=ones(size(xsg));

  -- Apply the transformation given current roll and pitch
  T = transpose(roty(IMU.data.pitch)*rotx(IMU.data.roll));
  --T = eye(4);
  X = [xsg ysg zsg onesg];
  Y=X*T;  --reverse the order because of transpose


  -- Do not use the points that supposedly hit the ground (check!!)
  indGood = Y(:,3) > -0.3;

  -- Separate coordinates in the sensor frame
  xsss = Y(indGood,1);
  ysss = Y(indGood,2);
  zsss = Y(indGood,3);
  onez = ones(size(xsss));

  -- These are now the default cartesian points
  LIDAR0.xs = xsss;
  LIDAR0.ys = ysss;

  --number of poses in each dimension to try

  --
  -- Begin the scan matching to update the SLAM pose
  --
  -- if encoders are zero, don't move
  -- if(SLAM.odomChanged > 0)
  if true then
    SLAM.odomChanged = 0;

    -- figure out how much to search over the yaw space based on the 
    -- instantaneous angular velocity from imu

    -- Perfrom the scan matching
    slamScanMatchPass1;
    slamScanMatchPass2;

    -- If no good fits, then use pure odometry readings
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
  T = transpose(trans([SLAM.x SLAM.y SLAM.z])*rotz(SLAM.yaw)*trans([LIDAR0.offsetx LIDAR0.offsety LIDAR0.offsetz]));
  X = [xsss ysss zsss onez];
  Y=X*T;  --reverse the order because of transpose

  -- Separate cartesian coordinates
  xss = Y(:,1);
  yss = Y(:,2);

  -- Convert each cartesian point to a map index
  xis = ceil((xss - OMAP.xmin) * OMAP.invRes);
  yis = ceil((yss - OMAP.ymin) * OMAP.invRes);

  -- Only keep point whose indices lie within the map boundaries
  indGood = (xis > 1) & (yis > 1) & (xis < OMAP.map.sizex) & (yis < OMAP.map.sizey);
  inds = sub2ind(size(OMAP.map.data),xis(indGood),yis(indGood));

  -- By how much should should we update the map
  -- at each index from a laser return?
  inc=5;
  if (SLAM.lidar0Cntr == 1)
    inc=100;
  end
  OMAP.map.data(inds)=OMAP.map.data(inds)+inc;
  DHMAP.map.data(inds) = 1;

  -- Decay the map around the robot
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
    end then
    if yimax > OMAP.map.sizey then
      yimax= OMAP.map.sizey;
    end

    -- Perform the decay on the surreoundings
    localMap = OMAP.map.data(ximin:ximax,yimin:yimax);
    indd = localMap<50 & localMap > 0;
    localMap(indd) = localMap(indd)*0.95;
    localMap(localMap>100) = 100;
    -- merge the small map back into the full map
    OMAP.map.data(ximin:ximax,yimin:yimax) = localMap;

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
  if (SLAM.x - OMAP.xmin < MAPS.edgeProx), xShift = -shiftAmount; end
  if (SLAM.y - OMAP.ymin < MAPS.edgeProx), yShift = -shiftAmount; end
  if (OMAP.xmax - SLAM.x < MAPS.edgeProx), xShift = shiftAmount; end
  if (OMAP.ymax - SLAM.y < MAPS.edgeProx), yShift = shiftAmount; end

  -- Perform the shift via helper functions
  if xShift ~= 0 or yShift ~= 0 then
    OMAP  = mapResize(OMAP,xShift,yShift);
    ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
  end

  -- Set the last updated time
  LIDAR0.lastTime = LIDAR0.scan.startTime;

end
