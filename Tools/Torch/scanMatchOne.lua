function scanMatchOne( LIDAR0, OMAP, xs, ys )

	SLAM = {}
	SLAM.xOdom = 0
	SLAM.yOdom = 0
	SLAM.yawOdom = 0

  -- Number of yaw positions to check
  nyaw1 = 15;
  -- At this resolution
  dyaw1 = 1.0 * math.pi/180.0;

  -- resolution of the candidate poses
  -- TODO: make this dependent on angular velocity / motion speed

  --tEncoders = ENCODERS.counts.t;
  tLidar0   = LIDAR0.startTime;

  --if abs(tLidar0-tEncoders) < 0.1
  nxs1  = 5;
  nys1  = 5;
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
  aCand1 = torch.range(-yawRange1,yawRange1)*dyaw1 + SLAM.yawOdom;
	-- + IMU.data.wyaw*0.025;
  xCand1 = torch.range(-xRange1,xRange1)*dx1 + SLAM.xOdom;
  yCand1 = torch.range(-yRange1,yRange1)*dy1 + SLAM.yOdom;

  -- get a local 3D sampling of pose likelihood
  hits = Slam.ScanMatch2D(
	'match',
	OMAP.data:storage(),
--	OMAP.data,
	OMAP.data:dim(),
	OMAP.sizex, OMAP.sizey,
--  xs:nElement(),
--  ys:nElement(),
	xs:storage(),--xsss(1:2:end),
	-- TODO: unfold to mean the 1:2:end syntax?
	ys:storage(),--ysss(1:2:end),
  xCand1,yCand1,aCand1
	);

  -- Find maximum
	hits_all = torch.resize( hits, hits:nElement() )
  hmax, imax = torch.max(hits,1);
	hmax = hmax[1];
	-- TODO: Find ind2sub replacement
  --kmax, mmax, jmax = ind2sub({nxs1,nys1,nyaw1},imax[i] );
	for k=1,nxs1 do
		for m=1,ny1 do
			for j=1,nyaw1 do
				if hits[{k,m,j}]==hmax then
					print('Found it!')
					kmax, mmax, jmax = k,m,j;
				end
			end
		end
	end

  if (SLAM.lidar0Cntr > 1) then

    -- Extract the 2D slice of xy poses at the best angle
    --hitsXY = hits(:,:,jmax);
		hitsXY = hits:select(3,jmax)

    -- Create a grid of distance-based costs from each cell to odometry pose
    yGrid1, xGrid1 = meshgrid( yCand1, xCand1 );
    xDiff1 = xGrid1 - SLAM.xOdom;
    yDiff1 = yGrid1 - SLAM.yOdom;
    distGrid1 = (1/3)*1e6*torch.sqrt(xDiff1:pow(2) + yDiff1:pow(2) );
    --distGrid1(abs(distGrid1
    minIndX, indx = torch.min( xCand1:add(-SLAM.xOdom):abs() );
    minIndY, indy = torch.min( yCand1:add(-SLAM.yOdom):abs() );

    -- Combine the pose likelihoods with the distance from odometry prediction
    -- TODO: play around with the weights!!
    --[[
    %costGrid1 = distGrid1 - hitsXY;
    %figure(2);
    %surf(yGrid1,xGrid1,distGrid1); % ~ 0 to 6*10^4 @ 1e6 muliplier
    %figure(3);
    %surf(yGrid1,xGrid1,hitsXY); % ~ 1*10^4 to 3*10^4
    --]]
    -- How valuable is the odometry preidiction?
    -- Should make a gaussian depression around this point...
    costGrid1 = - hitsXY;
    costGrid1[indx][indy] = costGrid1[indx][indy] - 500;
    --costGrid1(indx,indy) = costGrid1(indx,indy) - 2e4;

    -- Find the minimum and save the new pose
    cmin, cimin = torch.min( costGrid1:resize( costGrid1:nElement() ) );

    -- print( 'distGrid min %f, hits xy min =%f\n',
		-- distGrid1(cimin), hitsXY(cimin) );

    -- Save the best pose
    SLAM.yaw = aCand1(jmax);
    SLAM.x   = xGrid1(cimin);
    SLAM.y   = yGrid1(cimin);
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
