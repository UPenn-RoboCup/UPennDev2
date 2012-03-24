module(..., package.seeall);

require('PoseFilter');
require('Filter2D');
require('Config');	
require('Body')
require('shm');
require('vcm');
require('unix'); -- Get Time
require('wcm');
require('mcm');

--Should be checked for compatibility
enable_occmap = Config.vision.enable_freespace_detection or 0;

nCol = Config.camera.width/2/Config.vision.scaleB;
Div = Config.occmap.div or 72; --Fixed
Interval = 2*math.pi/Div;
HalfInter = Interval/2;
occDa = 2.5*math.pi/180;
occDr = 0.1;

-- IMU based Navigation
imuYaw = Config.world.imuYaw or 0;
yaw0 = 0;
yawAcc = 0;
xAcc = 0;
yAcc = 0;
uOdometry0 = vector.new({0, 0, 0});
odomScale = Config.world.odomScale;

occFilter = {};

occmap = {};
occmap.t = vector.zeros(Div);
occmap.x = {};
occmap.y = {};
occmap.r = vector.zeros(Div);

function entry()
  if enable_occmap ==0 then return; end

  print("initial occmap");
  for i = 1,Div do
    occFilter[i] = Filter2D.new();
	occFilter[i]:observation_ra(0.0,0.0,occDa,occDr);
  end
end

function getIdx(theta)

  theta = -theta;
  if theta < 0 then theta = theta + 2*math.pi; end
  theta = theta + HalfInter;
  local fanIdx = math.floor((theta-theta%Interval)/Interval + 0.5)+1;
  if fanIdx > Div then fanIdx = fanIdx - 1; end
  return fanIdx
end

function updateOdometry()
  uOdometry, uOdometry0 = mcm.get_odometry(uOdometry0);
  uOdometry[1] = odomScale[1]*uOdometry[1];
  uOdometry[2] = odomScale[2]*uOdometry[2];
  xAcc = xAcc + uOdometry[1];
  yAcc = yAcc + uOdometry[2]; 
  if (math.abs(xAcc)>=0.01) or (math.abs(yAcc)>=0.01) then
    for i = 1,Div do
      local xyOcc = vector.new({0,0});
      xyOcc[1],xyOcc[2] = occFilter[i]:get_xy();
      xyOcc[1] = xyOcc[1] - xAcc;
      xyOcc[2] = xyOcc[2] - yAcc;
      occFilter[i]:observation_xy(xyOcc[1],xyOcc[2],occDr,occDa);
    end
    xAcc = 0.0;
    yAcc = 0.0;
  end
  
--  print(unpack(uOdometry));
  if imuYaw == 1 then
    yaw = Body.get_sensor_imuAngle(3);
    --print("Body Yaw:",yaw*180/math.pi, " odom3 ",(yaw-yaw0)*180/math.pi);
    yawAcc = yawAcc + (yaw-yaw0);
    if (math.abs(yawAcc) >= Interval) then
      -- Rotation Update
      shiftDir = yawAcc/math.abs(yawAcc);
      if (shiftDir == 1) then
        stpin = 1; edpin = Div; inc = 1;
      else
        stpin = Div; edpin = 1; inc = -1;
      end
      -- rotate occumap
      tempFilter = Filter2D.new();
      tempFilter = occFilter[stpin];
      for cnt = stpin, edpin-inc , inc do
        occFilter[cnt] = occFilter[cnt+inc];
      end 
      occFilter[edpin] = tempFilter; 
      yawAcc = 0;
    end
    yaw0 = yaw;
  end 
end

function timeDecay()

  --occmap.t = wcm.get_occmap_t();
  for i = 1,Div do
    t1 = Body.get_time();
    dt = t1 - occmap.t[i];
    local raOcc = vector.new({0,0});
    raOcc[1],raOcc[2] = occFilter[i]:get_ra();
    if (raOcc[1] > 0.05) then
      raOcc[1] = math.min(10,raOcc[1] * (math.log(dt+1)/20+1));
      occFilter[i]:observation_ra(raOcc[1],raOcc[2],occDa,occDr);
    end
  end
end

function update()
  if enable_occmap ==0 then return; end

  timeDecay();
  updateOdometry();

  if (vcm.get_freespace_detect() == 1) then
    local freeBound = vcm.get_freespace_vboundB();
    local freeType = vcm.get_freespace_tboundB();
	-- Filter Update
    for i = 1,nCol do
      local v = vector.new({freeBound[i],freeBound[i+nCol]});
	  local r = math.sqrt(v[1]^2 + v[2]^2);
      local a = math.atan2(v[2], v[1]);
      local idx = getIdx(a);
      local raOcc = vector.new({0,0});
      raOcc[1],raOcc[2] = occFilter[idx]:get_ra();
      local update = true;
      if (freeType[i]==2) and (r<raOcc[1]) then
	    update = false;
      end
      if (freeType[i]==3) and (r>raOcc[1]) then
	    update = false;
      end
      -- record update timestamp
      if update then
        occmap.t[idx] = Body.get_time();
	    occFilter[idx]:observation_ra(r,a,occDa,occDr);
      end
    end
  end
  for i = 1,Div do
	  local raOcc = vector.new({0,0});
      raOcc[1],raOcc[2] = occFilter[i]:get_ra();
      occmap.r[i] = raOcc[1];
  end
  wcm.set_occmap_t(occmap.t);
  wcm.set_occmap_r(occmap.r);
end
