-- libSlam
-- (c) 2013 Stephen McGill
-- Perform SLAM given filtered laser points

-- Timing
require 'unix'
-- C Module for speed
local slam = require('slam')
-- Trig library
local libTrig = require'libTrig'

-- Configuration
-- TODO: Should be Config_Slam
local Sensors = require 'Config_Sensors'

require 'torch'
torch.Tensor = torch.DoubleTensor

local libSlam = {}

--Flag for IMU:
local IMUflag = true 
--local IMUflag = false 

-- Flag for benchmarking
local Benchmark = false --true
--local Benchmark = true

-- Default SLAM Map values
local MAPS = {}
--MAPS.res        = .1;
MAPS.res        = .05;
MAPS.invRes     = 1/MAPS.res;
MAPS.windowSize = 10; -- meters to see 
--MAPS.windowSize = 31; -- meters to see 
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
OMAP.data = torch.ByteTensor(OMAP.sizex,OMAP.sizex):fill(127) -- 127? 
OMAP.data_update = torch.DoubleTensor(OMAP.sizex,OMAP.sizex):zero()
OMAP.timestamp = unix.time();
libSlam.OMAP = OMAP;
print('Map size:',libSlam.MAPS.sizex,libSlam.MAPS.sizey)


-- Height Map
local HMAP = {}
HMAP.res    = MAPS.res;
HMAP.invRes = MAPS.invRes;
HMAP.xmin   = MAPS.xmin;
HMAP.ymin   = MAPS.ymin;
HMAP.xmax   = MAPS.xmax;
HMAP.ymax   = MAPS.ymax;
HMAP.zmin   = MAPS.zmin;
HMAP.zmax   = MAPS.zmax;
HMAP.sizex  = MAPS.sizex;
HMAP.sizey  = MAPS.sizey;
HMAP.hmax = 2
HMAP.hmin = -0.1
HMAP.data = torch.ByteTensor(HMAP.sizex,HMAP.sizex):fill(0) -- 127? 
HMAP.data_update = torch.DoubleTensor(HMAP.sizex,HMAP.sizex):zero()
HMAP.timestamp = unix.time();
libSlam.HMAP = HMAP;


-- Super Map
local SMAP = {}
SMAP.res    = MAPS.res
SMAP.invRes = MAPS.invRes
SMAP.xmin   = MAPS.xmin
SMAP.ymin   = MAPS.ymin
SMAP.xmax   = MAPS.xmax
SMAP.ymax   = MAPS.ymax
SMAP.zmin   = MAPS.zmin
SMAP.zmax   = MAPS.zmax
SMAP.sizex  = MAPS.sizex
SMAP.sizey  = MAPS.sizey
SMAP.hmax = 1
SMAP.hmin = -0.1
SMAP.data = torch.ByteTensor(SMAP.sizex,SMAP.sizex):fill(127) -- 127? 
SMAP.data_update = torch.DoubleTensor(SMAP.sizex,SMAP.sizex):zero()
SMAP.timestamp = unix.time()
libSlam.SMAP = SMAP


-- Setup SLAM values
local SLAM = {}
SLAM.x = 0;
SLAM.y = 0;
SLAM.z = 0;
SLAM.yaw = 0;
SLAM.xOdom = 0
SLAM.yOdom = 0
SLAM.yawOdom = 0
SLAM.lidar0Cntr = 0
SLAM.lidar1Cntr = 0
libSlam.SLAM = SLAM

-- Setup IMU values: euler angles are in radius already
local IMU = {}
IMU.roll = 0
IMU.pitch = 0
IMU.yaw = 0
IMU.dyaw = 0
IMU.lastYaw = 0
libSlam.IMU = IMU

---------------------------
-- For ThorCentaur
---------------------------
-- Setup Encoder values
local Encoder = {}
Encoder.dia = 0.1   -- REAL ROBOT:0.15
-- GAZEBO: BEST BASELINE 0.55 ~ 0.6
Encoder.baseline = 0.555  -- REAL ROBOT:0.34
Encoder.t_last = unix.time()
-- Scaler for encoder
Encoder.scaler = 1 
Encoder.l = 0
Encoder.r = 0
Encoder.cnts = 0
libSlam.Encoder = Encoder
---------------------------


-- Set the parameters for our C program
slam.set_resolution( MAPS.res )
slam.set_boundaries( OMAP.xmin, OMAP.ymin, OMAP.xmax, OMAP.ymax )

--print("Contiguous",xs:isContiguous(),ys:isContiguous())

local scan_match_tune = {}
local pass1 = {};
-- Number of yaw positions to check
pass1.nyaw = 1 --13 -- FIXME
pass1.dyaw = 0.5 * math.pi/180.0
-- At this resolution
-- TODO: make this dependent on angular velocity / motion speed
--if abs(tLidar0-tEncoders) < 0.1
pass1.nxs  = 19  --21
pass1.nys  = 19
-- resolution of the candidate poses
pass1.dx  = 0.05 --0.025
pass1.dy  = 0.05 --0.025
--else
--  nxs1  = 11;
--  nys1  = 11;
--  dx1   = 0.05;
--  dy1   = 0.05;
--end

-- Create the candidate locations in each dimension
pass1.xCand = torch.range(-1*math.floor(pass1.nxs/2), math.floor(pass1.nxs/2) )
pass1.yCand = torch.range(-1*math.floor(pass1.nys/2), math.floor(pass1.nys/2) )
pass1.aCand = torch.range(-1*math.floor(pass1.nyaw/2),math.floor(pass1.nyaw/2))
pass1.hits = torch.DoubleTensor( 
pass1.aCand:nElement(), pass1.xCand:nElement(), pass1.yCand:nElement()
):fill(0)
scan_match_tune[1] = pass1;

local pass2 = {};
pass2.nyaw = 15
pass2.dyaw = 0.05 * math.pi/180.0  --0.01
pass2.nxs  = 1
pass2.nys  = 1
pass2.dx   = 0.01
pass2.dy   = 0.01
pass2.xCand = torch.range(-1*math.floor(pass2.nxs/2), math.floor(pass2.nxs/2) )
pass2.yCand = torch.range(-1*math.floor(pass2.nys/2), math.floor(pass2.nys/2) )
pass2.aCand = torch.range(-1*math.floor(pass2.nyaw/2),math.floor(pass2.nyaw/2))
pass2.hits = torch.DoubleTensor( 
pass2.aCand:nElement(), pass2.xCand:nElement(), pass2.yCand:nElement()
):fill(0)
scan_match_tune[2] = pass2;

-- Instantiate to be big enough if all lidar hits are considered
-- These tensors hold (x,y,z,t) points for each lidar point
-- Each undergoes some different type of transformation
local W = torch.Tensor( Sensors.LIDAR0.nRays, 4 ):fill(0)
local Y = torch.Tensor( Sensors.LIDAR0.nRays, 4 ):fill(0)

-- Process lidar readings as the come in
-- Y are the transformed points in local x y z coordinates
libSlam.processL0 = function( lidar_points )
	if Benchmark then
		print('------------------')
		print('Process L0')
	end

	Y:resize( lidar_points:size(1), 4 )
	Y:copy(lidar_points)
	W:resize( Y:size(1), 4 )

	-- Increment counter
	SLAM.lidar0Cntr = SLAM.lidar0Cntr + 1

	----------------------
	-- Begin the scan matching to update the SLAM pose
	----------------------
  local scan_t = unix.time();
	-- if encoders are zero, don't move
	-- if(SLAM.odomChanged > 0)
	if true then
		local hmax = libSlam.scanMatchOne( Y )
		hmax = libSlam.scanMatchTwo( Y )
		
		-- If no good fits, then use pure odometry readings
		if hmax < 500 then  -- 500
			print('No good laser correlations! hmax:',hmax)
			SLAM.x = SLAM.xOdom
			SLAM.y = SLAM.yOdom
			if not IMUflag then
			    SLAM.yaw = SLAM.yawOdom
			else
			    SLAM.yaw = IMU.yaw
			end
			--print('PURE ODOMETRY: ', SLAM.xOdom, SLAM.yOdom, SLAM.yawOdom)
		end
	else
		print('Not moving');
	end
  if Benchmark then
		print(string.format('scan matching takes: \t\t%.5f ms', (unix.time()-scan_t)*1000) )
    scan_t = unix.time();
  end
	
	------------------------------------
	-- Update the SLAM odometry based on the scan matching
	------------------------------------
	SLAM.xOdom   = SLAM.x;
	SLAM.yOdom   = SLAM.y;
	SLAM.yawOdom = SLAM.yaw;
	
	------------------------------------
	-- Transform lidar points into the world frame
	------------------------------------
	local tmp = torch.mm( 
	libTrig.trans( {SLAM.x, SLAM.y, SLAM.z} ),
	libTrig.rotz( SLAM.yaw ) 
	)
	local T = torch.mm( 
	tmp, libTrig.trans( {0,0,0}
	))
	-- Perform the multiply
	W:mm( Y, T:t() )
  if Benchmark then
		print(string.format('lidar points transform update takes: \t\t%.5f ms', (unix.time()-scan_t)*1000) )
    scan_t = unix.time();
  end
	------------------------------------
	
	------------------------------------
	-- Set map update increment
	local inc = 5
	if SLAM.lidar0Cntr == 1 then
		inc = 100
	end
	------------------------------------

	------------------------------
	-- Perform the map update (15ms before)
	------------------------------
	OMAP.data_update:fill(0)
	slam.update_map( OMAP.data, W, inc, OMAP.data_update )


	OMAP.timestamp = unix.time()
  if Benchmark then
		print(string.format('map update update takes: \t\t%.5f ms', (unix.time()-scan_t)*1000) )
    scan_t = unix.time();
  end

	------------------------------
	-- Decay the map around the robot
	------------------------------
	
	local t0 = unix.time()	
	if SLAM.lidar0Cntr%20 == 0 then
	--[[
	--TODO: should use only a small window when exploring large-scale env
	
		-- Get the map indicies for the robot
		xiCenter = math.ceil((SLAM.x - OMAP.xmin) * OMAP.invRes);
		yiCenter = math.ceil((SLAM.y - OMAP.ymin) * OMAP.invRes);

		-- Amount of the surrounding to decay
		--windowSize = 30 * OMAP.invRes;
		windowSize = 100 * OMAP.res;
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
			yimin = 1;
		end
		if yimax > OMAP.sizey then
			yimax = OMAP.sizey;
		end

		-- Perform the decay on the surroundings
		--localMap = OMAP.data:sub( ximin,ximax,  yimin,yimax )
	--]]
	--print('I am here')
		
		
		---------------------------------------------------------------
		-- !!!!!TODO: TUNE PARAMETERS TODO BETTER DECAY!!!!
		--slam.decay_map(OMAP.data, 0, 256, 0.95, 240)
		---------------------------------------------------------------
		
		
		-- Merge the small map back into the full map
		--OMAP.data:sub( ximin,ximax,   yimin,yimax ):copy( localMap );
		OMAP.timestamp = unix.time();
	end
	
	
	----------------
	-- Bechmark
	----------------
	if Benchmark then
		print(string.format('map decay takes: \t\t%.5f ms', (unix.time()-t0)*1000) )
	end
	

	---------------------------
	-- Shift the map if needed
	---------------------------
	shiftAmount = 100*OMAP.res-- Always shift by a standard amount (in meters)
	xShift = 0
	yShift = 0
	-- Check in which directions we need to shift
	if (SLAM.x - OMAP.xmin < MAPS.edgeProx) then xShift = -shiftAmount
	elseif (OMAP.xmax - SLAM.x < MAPS.edgeProx) then xShift = shiftAmount
	end
	if (SLAM.y - OMAP.ymin < MAPS.edgeProx) then yShift = -shiftAmount
	elseif (OMAP.ymax - SLAM.y < MAPS.edgeProx) then yShift = shiftAmount
	end
	
	libSlam.xShift = xShift
	libSlam.yShift = yShift
	
	-- Perform the shift via helper functions
	if xShift ~= 0 or yShift ~= 0 then
		libSlam.mapShift(OMAP,xShift,yShift)
		libSlam.mapShift(HMAP,xShift,yShift)
		libSlam.mapShift(SMAP,xShift,yShift)
		slam.set_boundaries( OMAP.xmin, OMAP.ymin, OMAP.xmax, OMAP.ymax )
		slam.set_boundaries( HMAP.xmin, HMAP.ymin, HMAP.xmax, HMAP.ymax )
		slam.set_boundaries( SMAP.xmin, SMAP.ymin, SMAP.xmax, SMAP.ymax )
	end
	-- TODO: use meta.t from vcm
	-- Set the last updated time
	--LIDAR0.lastTime = LIDAR0.scan.startTime;

  if Benchmark then
		print('------------------')
  end
end

libSlam.scanMatchOne = function( Y )

	-- The first pass sets up variables
	-- and does not attempt to match scans
	if SLAM.lidar0Cntr <= 1 then
		-- TODO: add this function
		--xStart, yStart, thStart = FindStartPose(SLAM.x, SLAM.y, SLAM.yaw, xsss,ysss);
		print('First LIDAR pass!')
		print('Start: ', xStart)
		if true then
			--SLAM.x = xStart;
			--SLAM.y = yStart;
			--SLAM.yaw = Sensors.IMU.data.Y*math.pi/180*-1;--thStart;
			--SLAM.xOdom = xStart;
			--SLAM.yOdom = yStart;
			--SLAM.yawOdom = Sensors.IMU.data.Y*math.pi/180*-1;--thStart;
			
			SLAM.x = 0;
			SLAM.y = 0;
			SLAM.xOdom = 0;
			SLAM.yOdom = 0;
			if not IMUflag then
			    SLAM.yaw = 0
			    SLAM.yawOdom = 0
			else
			    SLAM.yaw = IMU.yaw
			    SLAM.yawOdom = IMU.yaw
			end
		end
		return 0;
	end

	-- Update the timestamps
	--tEncoders = ENCODERS.counts.t;
	tLidar0 = Sensors.LIDAR0.startTime;
    local wyaw = 0
    if IMUflag then
        wyaw = IMU.dyaw
    end
	--print('wyaw:',wyaw)

	-- Reset the ranges based on the current odometry
	-- TODO: determine how much to search over the yaw space based on 
	-- the instantaneous angular velocity from the imu
	local xCand = pass1.xCand;
	local yCand = pass1.yCand;
	local aCand = pass1.aCand;
	local hits = pass1.hits;
	xCand:range(-1*math.floor(pass1.nxs/2), math.floor(pass1.nxs/2) ):mul(pass1.dx)
	yCand:range(-1*math.floor(pass1.nys/2), math.floor(pass1.nys/2) ):mul(pass1.dy)
	aCand:range(-1*math.floor(pass1.nyaw/2),math.floor(pass1.nyaw/2)):mul(pass1.dyaw)
	xCand:add(SLAM.xOdom);
	yCand:add(SLAM.yOdom);
	aCand:add(SLAM.yawOdom):add( wyaw );
	
	-------------------------------
	-- Zero the hits, which will be accumulated in ScanMatch2D
	hits:zero()
	-- Prior on the middle location
	local midx = math.floor(xCand:size(1)/2)+1
	local midy = math.floor(yCand:size(1)/2)+1
	local mida = math.floor(aCand:size(1)/2)+1
	hits[mida][midx][midy] = 3000 --1000
	-- TODO: General Gaussian centered here
	-------------------------------
	
	-------------------------------
	-- Perform scan matching to find best pose
	-------------------------------
	local t_m0 = unix.time()
	local hmax, ixmax, iymax, iamax = slam.match(
	OMAP.data,
	Y, -- Transformed points
	xCand, yCand, aCand, 
	hits
	);
	SLAM.x = xCand[ixmax]
	SLAM.y = yCand[iymax]
	SLAM.yaw = aCand[iamax]
	--print('Match1', (unix.time()-t_m0)*1000,'ms' )
	print('ixmax, iymx, iamax', ixmax, iymax, iamax)
	
	return hmax
end

-----------------------------
-------- ScanMatchTwo -------
-----------------------------
libSlam.scanMatchTwo = function( Y )
	-- Reset the ranges based on the current odometry
	-- TODO: determine how much to search over the yaw space based on 
	-- the instantaneous angular velocity from the imu

  
	pass2.dyaw = math.abs(IMU.yawdot* 0.5)
	print('yawdot', IMU.yawdot)

	local xCand = pass2.xCand;
	local yCand = pass2.yCand;
	local aCand = pass2.aCand;
	local hits  = pass2.hits;
	xCand:range(-1*math.floor(pass2.nxs/2), math.floor(pass2.nxs/2) )
	yCand:range(-1*math.floor(pass2.nys/2), math.floor(pass2.nys/2) )
	aCand:range(-1*math.floor(pass2.nyaw/2),math.floor(pass2.nyaw/2))
	xCand:mul(pass2.dx):add(SLAM.x);
	yCand:mul(pass2.dy):add(SLAM.y);
	aCand:mul(pass2.dyaw):add(SLAM.yaw);
	
	-------------------------------
	-- Zero the hits, which will be accumulated in ScanMatch2D
	hits:zero()
	-------------------------------
	-- Prior on the middle location
	local midx = math.floor(xCand:size(1)/2)+1
	local midy = math.floor(yCand:size(1)/2)+1
	local mida = math.floor(aCand:size(1)/2)+1
	hits[mida][midx][midy] = 200
	-- TODO: General Gaussian centered here
	-------------------------------

	local t_m0 = unix.time()
	-- Perform the scan matching
	local hmax, ixmax, iymax, iamax = slam.match(
	OMAP.data,
	Y, -- Transformed points
	xCand, yCand, aCand
	,hits
	);
	
	SLAM.x = xCand[ixmax]
	SLAM.y = yCand[iymax]
	SLAM.yaw = aCand[iamax];
	--print('Match2', (unix.time()-t_m0)*1000,'ms' )
	return hmax;
end


------------------------------------------------------------------------------
-- Process the readings from chest/vertical lidar
------------------------------------------------------------------------------
libSlam.processL1 = function( lidar_points )
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print('------------------')
		print('Process L1')
	end
	local t0 = unix.time()
	----------------------------
	
	local Y = lidar_points:clone()

	-- Increment counter?
	-- TODO: Do we need this counter?
	SLAM.lidar1Cntr = SLAM.lidar1Cntr + 1


	-- Select the partiular coordinates for filtering
	-- Coordinates of Lidar hit points in body frame
	-- Chest lidar movement is included in previous transformations!
	-- TODO: Find way to change the clone() to copy() for speedup
	local xs = Y:select(2,1):clone()
	local ys = Y:select(2,2):clone()
	local zs = Y:select(2,3):clone()
	
    --print(string.format('\n\n max xs: %.3f \n\n', xs:max()))
    -- Eucleadian distance in horizontal plane from the robot's position
	local rClip = torch.sqrt( torch.add( xs:cmul(xs), ys:cmul(ys) ) )

    --print(string.format('\n\n max rClip: %.3f \n\n', rClip:max()))
	----------------------------------
	-- Detect negative cliffs
	-- TODO: CURRENTLY THEY ARE NOT USED SINCE WE
	-- ASSUME THAT THE GROUND IS ONE PLANE AND
	-- NO CLIFFS EXIST....
	----------------------------------
	local zscpy = zs:clone()
	local rClipcpy = rClip:clone()
	local zDiff = torch.add( zscpy:sub(2, -1), zscpy:sub(1, -2):mul(-1) )
	local rDiff = torch.add( rClipcpy:sub(2,-1), rClipcpy:sub(1,-2):mul(-1) )

	local goodcnt = 0
	-- TODO: C !!!
	--[[
	for i = 1, zDiff:size(1) do
		if zDiff[i]< math.min(-0.05, -1*math.tan(15/180*math.pi)*rDiff[i]) then
			goodcnt = goodcnt + 1
			if goodcnt == 1 then
				local iFirstCliff = i
				break
			end
		end
	end
	--]]
	local iFirstCliff = iFirstCliff or 99999
	--print('iFristCliff:', iFirstCliff)
	----------------------------------
	
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('Time used before binStats: \t%.2f ms', (unix.time()-t0)*1000) )
	end
	local t0 = unix.time()
	----------------------------
	
	
	-- Bin information for running statistics
	--TODO: tune the parameters
	local xBinLength = 0.1
	local xMaxRange = 25
	local xBinMax = xMaxRange/xBinLength -- in Matlab, there is a round, does it matter?
	local xBin = torch.range(1, xBinMax):mul(xBinLength)


	----------------------------------
	-- Call binStats.cc file to calculate the statistics for each bin
  	-- "count", "mean",  "max", "min", "std",
	local binTable = torch.Tensor(xBinMax, 5):zero()
	local bins = torch.Tensor(rClip:size(1),1):fill(-1)
	slam.binStats(rClip:div(xBinLength), zs, xBinMax, binTable, bins)
	----------------------------------
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('binStats takes: \t\t%.2f ms', (unix.time()-t0)*1000))
	end
	local t0 = unix.time()
	----------------------------
	
    
	-- Get the statistics from the binStats
	local counts = binTable:select(2, 1)
	local zMean = binTable:select(2, 2)
	local zMax = binTable:select(2, 3)
	local zMax_copy = zMax:clone()
	local zMaxMin = torch.add(zMax_copy, binTable:select(2, 4):mul(-1))
	
	-- Initialize indices for last ground bin and first obstacle bin
	--local gndIdx = 0
	--local obsIdx = 0
	local iGnd = torch.range(1, counts:size(1)):fill(0)
	local iObs = torch.range(1, counts:size(1)):fill(0)
	
	
	--[[
	--TODO: ALL NUMBERS NEED TO BE TUNED!!!
	-- Check for i<=# or i># because the robot sees itself as obstacle ???
	for i = 1, counts:size(1) do
		
		-- DO NOT PRINT THESE THINGS OR IT WILL CAUSE A DELAY AND REDUCE ACCURACY
		-- OF FREE SPACE UPDATE BY A LOT!!!!
		--print(string.format('bin, counts, zMaxMin, zMean, upper, lower:\t %d\t %d\t %.2f\t %.2f\t %.2f\t %.2f'
		--	,i, counts[i], zMaxMin[i], zMean[i], 0.3*xBin[i]+0.05-0.90, -0.3*xBin[i]-0.05-0.90))
		
		--print('zMean:', zMean[i])
		if counts[i]>=1 and zMaxMin[i]<=0.3 and zMean[i]<(0.3*xBin[i]+0.05-0.93)
		    and zMean[i]>(-0.3*xBin[i]-0.05-0.93) then
			gndIdx = gndIdx + 1
			iGnd[gndIdx] = i
			--print('Ground indices:', iGnd[gndIdx])
		--elseif (counts[i]>1 and zMaxMin[i]>0.08 and zMean[i]>(0.4*xBin[i]+0.10+0.90)) and i>15 then
		elseif counts[i]>1 and ( zMaxMin[i]>0.3) or zMean[i]>(0.4*xBin[i]+0.10-0.93 ) then
			obsIdx = obsIdx + 1
			iObs[obsIdx] = i
			--print('Walls indices:', iObs[obsIdx])	
		end
	end
	--]]
	

	
	---------------------------------------------------
	
	local gndIdx, obsIdx = slam.mask_points(counts:clone(), 
                          zMaxMin:clone(), zMean:clone(), xBin:clone(), iGnd, iObs)
	--print('iGnd iObs size: ', iGnd:size(1), iObs:size(1))
	--print('gndidx, obsidx', gndIdx, obsIdx)
	---------------------------------------------------
	
	
	
	---------------------------------------------------
	-- If there is no obstacle or cliff. 
	local obsFlag = 1	
	if obsIdx == 0 then
		iObs[1] = 0
		obsIdx = 1
		obsFlag = 0	 -- No obstacles
	end
	
	iGnd:resize(gndIdx, 1)  
	iObs:resize(obsIdx, 1)
	
	local iFirstObs = iObs[1][1]
	if iObs[1][1] == 0 then
		iFirstObs = xBinMax
	end
	
	--print('# Last Ground Bin: ', iGnd[gndIdx][1])
    --print('# First Obstacle Bin: ', iObs[1][1])
	--print('iFirstObs', iFirstObs)
	--------------------------------------------------
	
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('iGnd/iObs takes: \t\t%.2f ms', (unix.time()-t0)*1000, counts:size(1)))
	end
	local t0 = unix.time()
	----------------------------
	
	
	
	----------------------------------
	-- Deal with points that did not fit anywhere
	-- calculate the last free point before the 1st obstacle
	local last_point = 1
	local not_found_last_point = true
	
	-- Safe distance to be considered as free space from obstacle
	-- safe_distance x BinLengtn is in meters so
	-- safe_distance is measured in bins
	local safe_distance = 2	 
	---[[
	local cnt_obs = 0
	local O = torch.Tensor(bins:size(1), 4)
	local zMax = binTable:select(2, 3)
	--]]
	--[[
	for k=1, bins:size(1) do
		local i = bins:size(1)+1-k
		if bins[i][1]<1 then  
		    bins[i][1] = counts:size(1)
		end
		
		--print('bin #:', bins[i][1])
		--print('iFirstObs-safe_distance', iFirstObs-safe_distance)
		--print(not_found_last_point)
		
		if  (bins[i][1]> iFirstObs-safe_distance and not_found_last_point)  then
			last_point = k
			not_found_last_point = false
		end
		
		
		----------------------------------------------
		-- Take heights of all points for hmap
		O[i][1] = Y[i][1]
		O[i][2] = Y[i][2]
		O[i][3] = (Y[i][3] + 0.95 - HMAP.hmin)*(255/(HMAP.hmax - HMAP.hmin))--zMax[ bins[i][1] ] + 0.95--
		O[i][4] = 1
		--print(i,O[i][3],bins[i][1]  )
		
		----------------------------------------------	
	end
	--]]
	last_point = slam.get_last_free_point(bins, counts:size(1), iFirstObs, safe_distance)
	slam.get_height_points(O,Y,HMAP.hmin, HMAP.hmax, 0.95)
	
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('Post-binStats process takes: \t%.2f ms', (unix.time()-t0)*1000) )
	end
	local t0 = unix.time()
	----------------------------
	
	
	-- If no obstacle then the last free point is
	-- the last available point from the lidar scan
	if iObs[1][1] == 0 then
		last_point = bins:size(1)
	end
	
	if last_point ==1 then
    	--print(string.format('last point: \t %d', last_point))
    end
	----------------------------------
	
	-- Check which obstacle is first:
	-- An object or a cliff ?
	local iFirstBad = math.min( iFirstObs, iFirstCliff)
	
	
	-------------------------------------------------------
	-- Get the homogeneous free ground coordinates in body frame
	-------------------------------------------------------
	local G = torch.Tensor(last_point, 4)
	--[[	
	for jj = 1, last_point do
		local ii = Y:size(1)-jj + 1
		G[{ jj, {} }] = Y:select(1, ii)
	end
	--]]
	slam.get_ground_points(G,Y,last_point)
	--print('Last point: Size of G', last_point, G:size(1))
	
	--print('G Size: , ii:  bins size:', G:size(1) , temp_num, bins:size(1))
	
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('Get gnd points takes: \t\t%.2f ms', (unix.time()-t0)*1000))
	end
	local t0 = unix.time()
	----------------------------
	
	
	------------------------------------
	-- Transform homogeneous free ground  
	-- coordinates into the world frame
	------------------------------------
	-- Maybe not needed
	local T = torch.mm( 
	libTrig.trans( {SLAM.x, SLAM.y, SLAM.z} ),
	libTrig.rotz( SLAM.yaw )
	)
	local tmpG = torch.Tensor(G:size(1), 4)
	tmpG:mm( G, T:t() )
	------------------------------------
	
	
	------------------------------------
	-- Set map update increment
	local incObs = 100
	local incGnd = -100
	------------------------------------

	-------------------------------------------
	-- Perform the ground free space map update
	-------------------------------------------
	OMAP.data_update:fill(0)
	slam.update_map( OMAP.data, tmpG, incGnd, OMAP.data_update )
	
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('OMAP updating takes: \t\t%.2f ms', (unix.time()-t0)*1000) )
	end
	local t0 = unix.time()
	----------------------------
	
	-------------------------------------------
	-- Perform the obstacles height map update
	-------------------------------------------
	local tmpO = torch.Tensor(O:size(1), 4)
	tmpO:mm( O, T:t() )
	slam.update_hmap( HMAP.data, tmpO )
	
	----------------------------
	-- Benchmark
	if Benchmark then
		print(string.format('HMAP updating takes: \t\t%.2f ms', (unix.time()-t0)*1000) )
	end
	local t0 = unix.time()
	----------------------------
end
----------------------------------------------------



----------------------------------------------------
-- Integrate two maps
----------------------------------------------------
libSlam.mergeMap = function()
	slam.update_smap( OMAP.data, HMAP.data, SMAP.data )
end



----------------------------------------------------
-- Process the IMU data
----------------------------------------------------
local t_pre = 0
libSlam.processIMU = function( rpy, yawdot, t )
	IMU.roll = rpy[1]
	IMU.pitch = rpy[2]
	IMU.yaw = rpy[3]
	IMU.dyaw = IMU.yaw - IMU.lastYaw
	dt = t - t_pre
	--TODO: gyro readings are always zero
	IMU.yawdot = IMU.dyaw/dt
	t_pre = t
	IMU.lastYaw = IMU.yaw
end
----------------------------------------------------


----------------------------------------------------
-- Process the Encoder data
----------------------------------------------------
libSlam.processOdometry = function( encoder_lv, encoder_rv)
    -- Input should be POSITION
	
	--print(string.format('\n\ninput vel: %.3f \t %.3f\n\n', encoder_lv, encoder_rv))
	
	--[[
	-- The following is for situation where inputs are position
	-- Compute the distance of motion for both wheels
	local dl = (2*math.pi*cnts + encoder_l/180*math.pi) / 2 * Encoder.dia
	local dr = (2*math.pi*cnts + encoder_r/180*math.pi) / 2 * Encoder.dia
	
	-- Record the current readings
	Encoder.l = encoder_l
	Encoder.r = encoder_r
	Encoder.cnts  = cnts
	--]]
	
	

	-- The following is for situation where inputs are velocities
    -- Compute the distance of motion for both wheels
    local dt = unix.time() - Encoder.t_last
	Encoder.t_last = unix.time()
	local dl = encoder_lv*dt*Encoder.scaler / 2 * Encoder.dia
   	local dr = encoder_rv*dt*Encoder.scaler / 2 * Encoder.dia
	local dtheta = 0
	local dx = 0
	local dy = 0
	
	-- Check dl, dr
	if math.abs(dl - dr) < 0.0005 then
		--print('MOVING FORWARD!!!')
		dy = 0
		dx = 0.5 * ( dl + dr)
		dtheta = 0
	else
	
		--TODO: use IMU for dtheta	
		dtheta = math.atan2( (dr - dl), Encoder.baseline )
		

		--print(string.format('\n\n dt %.5f,\t dl: %.3f, \t dr: %.3f \n\n', dt, dl, dr))

		-- Find the pivot point (distance off the left wheel center)
		R = Encoder.baseline/2 + Encoder.baseline*dl/(dl-dr)

		-- Compute changes in pose
	 	dx = R * math.sin( dtheta )
		dy = R * ( 1 - math.cos( dtheta ) )
	  	
	end
	
	--print(string.format('\n\n dtheta, %.10f, \t dx: %.3f, \t dy: %.3f \n\n', dtheta, dx, dy))
	

	-- Update odometry
    SLAM.xOdom = SLAM.xOdom + dx
    SLAM.yOdom = SLAM.yOdom + dy
    SLAM.yawOdom = SLAM.yawOdom + dtheta
	
end
----------------------------------------------------


----------------------------------------------------
-- mapShift(myMap,x,y)
-- myMap: Map table to be copied.
-- x: Amount (in meters) to shift map by in the x direction
-- y: Amount (in meters) to shift map by in the y direction
-- Use: mapShift( OMAP, 5*OMAP.res, 5*OMAP.res )
libSlam.mapShift = function(myMap,x,y)
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
		yis  = torch.range( 1, myMap.sizey );
		nyis = torch.range( 1, myMap.sizey );
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
	'Map shifted. New bounding box: x(%f %f) y(%f %f)\n',
	myMap.xmin,myMap.xmax,myMap.ymin,myMap.ymax
	));

end
------------------------------------------------------------------------------

-- Yield the library
return libSlam
