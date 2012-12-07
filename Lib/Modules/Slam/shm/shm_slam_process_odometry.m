%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Encoder message handler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shm_slam_process_odometry(data,name)
global ODOMETRY SLAM IMU
persistent tLastUpdate stopCntr

if isempty(IMU.data)
    return
end

if isempty(tLastUpdate)
  tLastUpdate = 0;
end

if isempty(stopCntr)
  stopCntr=0;
end

if ~isempty(data)
  ODOMETRY.odomLast = ODOMETRY.odom;
  ODOMETRY.odom = data;
  ODOMETRY.cntr  = ODOMETRY.cntr + 1;
  
  if isempty(ODOMETRY.tLastReset)
    ODOMETRY.tLastReset = IMU.data.t;
    ODOMETRY.tLast = IMU.data.t;
    return;
  end
  
  %dt for velocity calculation
  dtv = IMU.data.t-ODOMETRY.tLastReset;
  if (dtv > 0.1)
    ODOMETRY.tLastReset = IMU.data.t;
  end
  
  dpose = ODOMETRY.odom-ODOMETRY.odomLast;
  
  %% Keep track of how long we remain still
  if sum( abs(dpose) )<0.001
    stopCntr = stopCntr + 1;
  else
    stopCntr = 0;
  end
  
  % if too still, then dont change the yaw
  if (stopCntr < 40)
    %wdt = IMU.data.wyaw * 0.025; %(GetUnixTime()-tLastUpdate);
    %wdt = IMU.data.wyaw * (IMU.data.t-tLastUpdate);
    wdt = 0;
  else
    wdt = 0;
    %fprintf(1,'not moving\n');
  end
  
  tLastUpdate = IMU.data.t;
  %dt = counts.t - ODOMETRY.tLast;
  
  xPrev   = SLAM.xOdom;
  yPrev   = SLAM.yOdom;
  yawPrev = SLAM.yawOdom;
  
  %this does not seem to do anything...
  %the idea is to project the displacement onto the 2D plane, given pitch
  %and roll
  dTrans       = rotz(SLAM.yaw)*roty(IMU.data.pitch)*rotx(IMU.data.roll)*rotz(SLAM.yaw)'*...
      [dpose(1);dpose(2);0;1]; 
  
  SLAM.xOdom   = xPrev   + dTrans(1);
  SLAM.yOdom   = yPrev   + dTrans(2);
  SLAM.yawOdom = yawPrev + dpose(3);

  %{
  if (abs(SLAM.xOdom-SLAM.x) > 0.00001 || abs(SLAM.yOdom-SLAM.y) > 0.00001 || abs(SLAM.yawOdom-SLAM.yaw) > 0.00001)
    SLAM.odomChanged = 1;
  end
  %}
  
end