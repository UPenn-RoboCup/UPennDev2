function [local_map global_map] = shm_slam()

global LIDAR IMU

init_slam();

%% Run the update
while 1
    IMU.data.roll = 0;
    IMU.data.pitch = 0;
    IMU.data.yaw = 0;
    myranges = LIDAR.get_ranges();
    data = {};
    data.ranges = myranges.ranges;
    data.startTime = myranges.t;
    IMU.data.t = myranges.t;;
    IMU.tLastArrival = myranges.t;;
    
    shm_slam_process_lidar0(data);
    pause(.1);
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

SLAM.explorationUpdateTime   = ROBOT.wcmRobot.get_time();
SLAM.plannerUpdateTime       = ROBOT.wcmRobot.get_time();
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

%initialize scan matching function
ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
ScanMatch2D('setResolution',OMAP.res);
ScanMatch2D('setSensorOffsets',[LIDAR0.offsetx LIDAR0.offsety LIDAR0.offsetz]);

end
