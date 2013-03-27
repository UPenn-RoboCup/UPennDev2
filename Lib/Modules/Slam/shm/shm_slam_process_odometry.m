%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Encoder message handler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shm_slam_process_odometry(name)
global ODOMETRY SLAM IMU POSE
persistent tLastUpdate stopCntr

if isempty(IMU.data)
    disp('No IMU data!');
    return;
end

if isempty(tLastUpdate)
    tLastUpdate = 0;
    ODOMETRY.odom = [0,0,0];
    ODOMETRY.odomLast = ODOMETRY.odom;
end

if isempty(stopCntr)
    stopCntr=0;
end
ODOMETRY.odomLast = ODOMETRY.odom;
ODOMETRY.odom = ODOMETRY.odomLast;%data.odometry;
ODOMETRY.cntr  = ODOMETRY.cntr + 1;

if isempty(ODOMETRY.tLastReset)
    ODOMETRY.tLastReset = IMU.data.t;
    ODOMETRY.tLast = IMU.data.t;
    disp('Setting the last timestep...');
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
wdt = 0;
if (stopCntr < 40)
    %wdt = IMU.data.wyaw * 0.025; %(GetUnixTime()-tLastUpdate);
    wdt = IMU.data.wyaw * (IMU.data.t-tLastUpdate);
    %wdt = 0;
else
    %wdt = 0;
    %fprintf(1,'not moving\n');
end
%fprintf(1,'wdt: %f\n', wdt);

tLastUpdate = IMU.data.t;
%dt = counts.t - ODOMETRY.tLast;

xPrev   = SLAM.xOdom;
yPrev   = SLAM.yOdom;
yawPrev = SLAM.yawOdom;

%this does not seem to do anything...
%the idea is to project the displacement onto the 2D plane, given pitch
%and roll
dTrans = rotz(SLAM.yaw)*roty(IMU.data.pitch)*rotx(IMU.data.roll)*rotz(SLAM.yaw)'*...
    [dpose(1);dpose(2);0;1];

SLAM.xOdom   = xPrev   + dTrans(1);
SLAM.yOdom   = yPrev   + dTrans(2);
SLAM.yawOdom = yawPrev + dpose(3); % This is good
%SLAM.yawOdom = yawPrev + wdt;
%SLAM.yawOdom = -1*IMU.data.yaw;

%% For using dead reckoning
POSE.data.x = SLAM.xOdom;
POSE.data.y = SLAM.yOdom;
POSE.data.yaw = SLAM.yawOdom;

%disp('Updating POSE...');
%{
  if (abs(SLAM.xOdom-SLAM.x) > 0.00001 || abs(SLAM.yOdom-SLAM.y) > 0.00001 || abs(SLAM.yawOdom-SLAM.yaw) > 0.00001)
    SLAM.odomChanged = 1;
  end
%}