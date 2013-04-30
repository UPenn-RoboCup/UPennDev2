%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Encoder message handler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamProcessEncoders(data,name)
global ENCODERS SLAM IMU
persistent tLastUpdate stopCntr

if isempty(IMU.data)
    return
end

if isempty(tLastUpdate)
  tLastUpdate = GetUnixTime();
end

if isempty(stopCntr)
  stopCntr=0;
end

if ~isempty(data)
  ENCODERS.counts  = MagicEncoderCountsSerializer('deserialize',data);
  ENCODERS.cntr    = ENCODERS.cntr + 1;
  
  if isempty(ENCODERS.tLastReset)
    ENCODERS.tLastReset = ENCODERS.counts.t;
    ENCODERS.tLast = ENCODERS.counts.t;
    return;
  end
  
  counts = ENCODERS.counts;
  ENCODERS.acounts = ENCODERS.acounts + [counts.fr;counts.fl;counts.rr;counts.rl];

  %dt for velocity calculation
  dtv = counts.t-ENCODERS.tLastReset;
  if (dtv > 0.1)
    ENCODERS.wheelVels = ENCODERS.acounts / dtv * ENCODERS.metersPerTic;
    ENCODERS.acounts = ENCODERS.acounts*0;
    ENCODERS.tLastReset = counts.t;
  end
  
  %{
  encr = [ENCODERS.counts.fr ENCODERS.counts.rr];
  encl = [ENCODERS.counts.fl ENCODERS.counts.rl];
  
  %get the mean travelled distance for left and right sides
  dr = abs(encr(1) - encr(2));
  dl = abs(encl(1) - encl(2));
  
  [drmin drimin] = min(abs(encr));
  [dlmin dlimin] = min(abs(encl));
  
  
  diffThresh = 2;
  %if the difference between wheel tics is greater than threshold, then use
  %the value with the minimum magnitude (the other wheel may be slipping)
  if (dr < diffThresh)
    rc = mean(encr) * ENCODERS.metersPerTic;
  else
    rc = encr(drimin) * ENCODERS.metersPerTic;
  end
  
  if (dl < diffThresh)
    lc = mean(encl) * ENCODERS.metersPerTic;
  else
    lc = encl(dlimin) * ENCODERS.metersPerTic;
  end
  %}
  rc = ENCODERS.counts.rr * ENCODERS.metersPerTic;
  lc = ENCODERS.counts.rl * ENCODERS.metersPerTic;
  
  if ( (ENCODERS.counts.rr == 0) && (ENCODERS.counts.rl == 0) )
    stopCntr = stopCntr + 1;
  else
    stopCntr = 0;
  end
  
  
  vdt = mean([rc,lc]);
  
  %the fudge factor scales the angular change due to slippage
  %TODO: this will also affect vdt!!
  %wdt = (rc - lc)/(2*ENCODERS.robotRadius*ENCODERS.robotRadiusFudge);
  
  
  if (stopCntr < 40)
    wdt = IMU.data.wyaw * 0.025; %(GetUnixTime()-tLastUpdate);
  else
    wdt = 0;
    vdt = 0;
    %fprintf(1,'not moving\n');
  end
  
  
  tLastUpdate = GetUnixTime();
  %dt = counts.t - ENCODERS.tLast;
  
  xPrev   = SLAM.xOdom;
  yPrev   = SLAM.yOdom;
  yawPrev = SLAM.yawOdom;
  
  %calculate the change in position
  if (abs(wdt) > 0.001)
    dx   = -vdt/wdt*sin(yawPrev) + vdt/wdt*sin(yawPrev+wdt);
    dy   =  vdt/wdt*cos(yawPrev) - vdt/wdt*cos(yawPrev+wdt);
    dyaw =  wdt;
  else
    dx   =  vdt*cos(yawPrev);
    dy   =  vdt*sin(yawPrev);
    dyaw =  wdt;
  end
  
  %this does not seem to do anything...
  %the idea is to project the displacement onto the 2D plane, given pitch
  %and roll
  dTrans       = rotz(SLAM.yaw)*roty(IMU.data.pitch)*rotx(IMU.data.roll)*rotz(SLAM.yaw)'*[dx;dy;0;1]; 
  
  SLAM.xOdom   = xPrev   + dTrans(1);
  SLAM.yOdom   = yPrev   + dTrans(2);
  SLAM.yawOdom = yawPrev + dyaw;

  %{
  if (abs(SLAM.xOdom-SLAM.x) > 0.00001 || abs(SLAM.yOdom-SLAM.y) > 0.00001 || abs(SLAM.yawOdom-SLAM.yaw) > 0.00001)
    SLAM.odomChanged = 1;
  end
  %}
  
end