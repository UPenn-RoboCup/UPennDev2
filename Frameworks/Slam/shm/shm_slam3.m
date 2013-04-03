function [local_map global_map] = shm_slam()

global LIDAR IMU OMAP SLAM MAPS LIDAR0

init_slam();
figure(1);
clf;

%% Run the update
while 1
    myranges = LIDAR.get_ranges();
    data = {};
    data.ranges = myranges.ranges;
    data.startTime = myranges.t;
    data.odometry = myranges.odom;
    IMU.data.roll = 0;
    IMU.data.pitch = 0;
    IMU.data.yaw = myranges.imu(3);%0;
    IMU.data.wyaw = myranges.gyro(3);
    IMU.data.t = myranges.t;
    IMU.tLastArrival = myranges.t;
    
    % Update encoders
    shm_slam_process_odometry(data);
    shm_slam_process_lidar0(data);
    
    % Simple plot
    figure(1);
    cla;
    subplot(2,1,1);
    image( OMAP.map.data' )
    colormap(gray)
    hold on;
    img_coords = MAPS.invRes * [SLAM.x,SLAM.y] + [MAPS.map.sizex MAPS.map.sizey]/2;
    plot(img_coords(1),img_coords(2),'y.','MarkerSize',20)
    xd = 25*cos(SLAM.yaw);
    yd = 25*sin(SLAM.yaw);
    quiver(img_coords(1),img_coords(2),xd,yd,'y');
    title(sprintf('x:%f, y:%f, yaw: %f',SLAM.xOdom,SLAM.yawOdom,SLAM.yaw*180/pi),'FontSize',12);
    
    subplot(2,1,2);
    cla;
    polar( LIDAR0.angles(:),myranges.ranges(:), 'r.' );
    hold on;
    xd = max(myranges.ranges)*cos(IMU.data.yaw);
    yd = max(myranges.ranges)*sin(IMU.data.yaw);
    compass(xd,yd);
    
    pause(.025);
end

end

function init_slam()
global SLAM OMAP POSE LIDAR0 REF_MAP START_POSITION ROBOT LIDAR

%% Initialize shared memory segment
disp('Initializing robot shm...')
if isempty(ROBOT)
ROBOT = shm_robot(22,2);
end
disp('Initializing lidar shm...')
if isempty(LIDAR)
LIDAR = shm_lidar(22,2);
end
disp('Loading the Config file')
loadConfig();

SLAM.updateExplorationMap    = 0;
SLAM.explorationUpdatePeriod = 5;
SLAM.plannerUpdatePeriod     = 2;
myranges = LIDAR.get_ranges();

SLAM.explorationUpdateTime   = myranges.t;
SLAM.plannerUpdateTime       = myranges.t;
poseInit();

SLAM.x            = POSE.xInit;
SLAM.y            = POSE.yInit;
SLAM.z            = POSE.zInit;
SLAM.yaw          = POSE.data.yaw;
SLAM.lidar0Cntr   = 0;
SLAM.lidar1Cntr   = 0;

%SLAM.IncMapUpdateHMsgName = GetMsgName('IncMapUpdateH');
%SLAM.IncMapUpdateVMsgName = GetMsgName('IncMapUpdateV');
%ipcAPIDefine(SLAM.IncMapUpdateHMsgName);
%ipcAPIDefine(SLAM.IncMapUpdateVMsgName);

SLAM.xOdom        = SLAM.x;
SLAM.yOdom        = SLAM.y;
SLAM.yawOdom      = SLAM.yaw;
SLAM.odomChanged  = 1;

SLAM.cMapIncFree = -5;
SLAM.cMapIncObs  = 10;
SLAM.maxCost     = 100;
SLAM.minCost     = -100;

% Initialize maps
initMapProps;
omapInit;        %localization map
emapInit;        %exploration map ??
cmapInit;        %vertical lidar map
dvmapInit;       %vertical lidar delta map
dhmapInit;       %horizontal lidar delta map

% Initialize data structures
imuInit;
lidar0Init;
odometryInit;

%initialize scan matching function
ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
ScanMatch2D('setResolution',OMAP.res);
ScanMatch2D('setSensorOffsets',[LIDAR0.offsetx LIDAR0.offsety LIDAR0.offsetz]);

end
