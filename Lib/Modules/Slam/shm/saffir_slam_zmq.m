clear all;
%% Set up variables
global SLAM OMAP POSE LIDAR0 REF_MAP START_POSITION ROBOT LIDAR MAPS IMU
s_laser = zmq('subscribe',5555);
s_imu = zmq('subscribe',5556);

%vidObj = VideoWriter('slam.avi');
%open(vidObj);

figure(1);
clf(gcf);
colormap(hot)
%subplot(2,1,1);
hold on;
hMap = [];
hPose = [];
hOrientation = [];
initSlam = [];
%{
hTitle = title(sprintf('x:%f, y:%f, yaw: %f', ...
        POSE.data.x,POSE.data.y,POSE.data.yaw*180/pi), ...
        'FontSize',12);
%}

%% Initialize structures
if isempty(initSlam)
    
    %% Initialize shared memory segment
    %{
    disp('Initializing robot shm...')
    if isempty(ROBOT)
        ROBOT = shm_robot(0,1);
    end
    disp('Initializing lidar shm...')
    if isempty(LIDAR)
        LIDAR = shm_lidar(0,1);
    end
    %}
    disp('Loading the Config file')
    loadConfig();
    
    SLAM.updateExplorationMap    = 0;
    SLAM.explorationUpdatePeriod = 5;
    SLAM.plannerUpdatePeriod     = 2;
    %myranges = LIDAR.get_ranges();
    myranges = {};
    myranges.t = 0;
    
    SLAM.explorationUpdateTime   = 0;
    SLAM.plannerUpdateTime       = 0;
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

%% Run the update
while 1
    %% Grab the data, and massage is for API compliance
    %myranges = LIDAR.get_ranges();
    [data,idx] = zmq('poll');
    for i=1:numel(idx)
        if idx(i)==s_laser
            newscan = msgpack('unpack', data{i});
            LIDAR0.scan.ranges = typecast( uint8(newscan.ranges), 'single' );
            LIDAR0.scan.startTime = double(newscan.startTime);
            shm_slam_process_lidar0();
        else
            imu = msgpack('unpack', data{i});
            IMU.data.roll = double(imu.R)*pi/180;
            IMU.data.pitch = double(imu.P)*pi/180;
            IMU.data.yaw = double(imu.Y)*pi/180;
            IMU.data.wyaw = -1*(double(imu.Wz)-370)*(pi/180.0/3.45);%double(imu.Wz);
            IMU.data.t = double( imu.t );
            IMU.tLastArrival = double(IMU.data.t);
            shm_slam_process_odometry();
        end
    end
    
    
    %% Run SLAM Update processes
    
    
    %% Simple plotting
    if isempty(hMap)
        hMap = imagesc( OMAP.map.data' );
        axis([0 OMAP.map.sizex 0 OMAP.map.sizey]);
    else
        set(hMap,'cdata',OMAP.map.data');
    end
    % Define how to plot the robot's pose
    xi = (POSE.data.x-OMAP.xmin)*OMAP.invRes; % x image coord
    yi = (POSE.data.y-OMAP.ymin)*OMAP.invRes; % y image coord
    xd = 25*cos( double(POSE.data.yaw) );
    yd = 25*sin( double(POSE.data.yaw) );
    if isempty(hOrientation) || isempty(hPose)
        hPose = plot(xi,yi,'g.','MarkerSize',20);
        hOrientation = quiver(xi,yi,xd,yd,'g');
    else
        set( hPose, 'xdata',xi, 'ydata',yi );
        set( hOrientation, 'xdata',xi, 'ydata',yi );
        set( hOrientation, 'udata',xd, 'vdata',yd );
    end
    
    hTitle = title(sprintf('x:%f, y:%f, yaw: %f', ...
        POSE.data.x,POSE.data.y,POSE.data.yaw*180/pi), ...
        'FontSize',12);
    
    %     subplot(2,1,2);
    %     cla;
    %     polar( LIDAR0.angles(:),data.ranges(:), 'r.' );
    %     hold on;
    %     xd = max(data.ranges)*cos(IMU.data.yaw);
    %     yd = max(data.ranges)*sin(IMU.data.yaw);
    %     compass(xd,yd);
    
    drawnow;
end