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

function scanMatchOne( SLAM, LIDAR0, OMAP, xs, ys )
  --tEncoders = ENCODERS.counts.t;
  tLidar0   = LIDAR0.startTime;
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
