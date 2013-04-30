function slam(addr,id)
global SLAM

SetMagicPaths;

if nargin < 1
  SLAM.addr = 'localhost';
else
  SLAM.addr = addr;
end

if nargin >1
  setenv('ROBOT_ID',sprintf('%d',id));
end

SLAM.useUdpExternal = 1;
SLAM.useIpcExternal = 0;

renice(-20);

slamStart;

while(1)
  slamUpdate;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize slam process
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamStart
global SLAM OMAP POSE LIDAR0 REF_MAP START_POSITION

SetMagicPaths;

loadConfig;

ipcAPIHandle = @ipcAPI;
%ipcAPIHandle = @ipcWrapperAPI; %use threaded version

ipcInit(SLAM.addr,ipcAPIHandle);


if (SLAM.useUdpExternal)
  % Broadcast address is bad over WiFi; direct to GCS
  
  masterIp = getenv('MASTER_IP');
  if isempty(masterIp)
    error('MASTER_IP env var is not defined');
  end
  masterPort = 12346;
  UdpSendAPI('connect',masterIp,masterPort);
end

SLAM.updateExplorationMap    = 0;
SLAM.explorationUpdatePeriod = 5;
SLAM.plannerUpdatePeriod     = 2;

SLAM.explorationUpdateTime   = GetUnixTime();
SLAM.plannerUpdateTime       = GetUnixTime();
poseInit();

SLAM.x            = POSE.xInit;
SLAM.y            = POSE.yInit;
SLAM.z            = POSE.zInit;
SLAM.yaw          = POSE.data.yaw;
SLAM.lidar0Cntr   = 0;
SLAM.lidar1Cntr   = 0;

SLAM.IncMapUpdateHMsgName = GetMsgName('IncMapUpdateH');
SLAM.IncMapUpdateVMsgName = GetMsgName('IncMapUpdateV');
ipcAPIDefine(SLAM.IncMapUpdateHMsgName);
ipcAPIDefine(SLAM.IncMapUpdateVMsgName);

SLAM.xOdom        = SLAM.x;
SLAM.yOdom        = SLAM.y;
SLAM.yawOdom      = SLAM.yaw;
SLAM.odomChanged  = 1;

SLAM.cMapIncFree = -5;
SLAM.cMapIncObs  = 10;
SLAM.maxCost     = 100;
SLAM.minCost     = -100;

%initialize maps
initMapProps;
omapInit;        %localization map
emapInit;        %exploration map ??
cmapInit;        %vertical lidar map
dvmapInit;       %vertical lidar delta map
dhmapInit;       %horizontal lidar delta map

%initialize data structures
imuInit;
lidar0Init;
lidar1Init;
servo1Init;
motorsInit;
gpsInit;

sendServoStart;

%load refMoore308.mat

%define messages
DefineSensorMessages;

if checkVis
  DefineVisMsgs;
end

%assign the message handlers
%arguments are (msgName, function handle, ipcAPI handle, queue length)
ipcReceiveSetFcn(GetMsgName('Lidar0'),      @slamProcessLidar0,   ipcAPIHandle,40);
ipcReceiveSetFcn(GetMsgName('Lidar1'),      @slamProcessLidar1_2, ipcAPIHandle,40);
ipcReceiveSetFcn(GetMsgName('Servo1'),      @slamProcessServo1,   ipcAPIHandle,40);
ipcReceiveSetFcn(GetMsgName('Encoders'),    @slamProcessEncoders, ipcAPIHandle,40);
ipcReceiveSetFcn(GetMsgName('ImuFiltered'), @slamProcessImu,      ipcAPIHandle,100);
ipcReceiveSetFcn(GetMsgName('GPS'),         @slamProcessGps,      ipcAPIHandle,10);

%initialize scan matching function
ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
ScanMatch2D('setResolution',OMAP.res);
ScanMatch2D('setSensorOffsets',[LIDAR0.offsetx LIDAR0.offsety LIDAR0.offsetz]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Receive and handle ipc messages
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamUpdate
ipcReceiveMessages;

