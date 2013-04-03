clear all;
%% Set up variables
global SLAM OMAP POSE LIDAR0 REF_MAP START_POSITION ROBOT LIDAR MAPS IMU
s_flir = zmq('subscribe',5555);
flimg = zeros(320,256);
s_laser = zmq('subscribe',5556);
s_imu = zmq('subscribe',5557);

%vidObj = VideoWriter('slam.avi');
%open(vidObj);

figure(1);
clf(gcf);
subplot(2,2,1);
colormap(hot);
hold on;
hMap = [];
hPose = [];
hOrientation = [];
initSlam = [];
hFlir = [];
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
    LIDAR0.scan.startTime = 0;
    odometryInit;
    
    %initialize scan matching function
    ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
    ScanMatch2D('setResolution',OMAP.res);
    ScanMatch2D('setSensorOffsets',...
        [LIDAR0.offsetx LIDAR0.offsety LIDAR0.offsetz]);
    
end

%% Run the update
while 1
    %% Grab the data, and massage is for API compliance
    %myranges = LIDAR.get_ranges();
    [data,idx] = zmq('poll');
    for i=1:numel(idx)
        if idx(i)==s_laser
            %return;
            %disp('laser')
            newscan = msgpack('unpack', data{i});
            LIDAR0.scan.ranges = typecast(uint8(newscan.ranges),'single')';
            LIDAR0.scan.startTime = double(newscan.startTime);
            %{
            LIDAR0.scan.ranges = typecast(data{1},'single');
            LIDAR0.scan.startTime = LIDAR0.scan.startTime + 1/40;
            %}
            shm_slam_process_lidar0();
        elseif idx(i)==s_imu
            %disp('imu')
            imu = msgpack('unpack', data{i});
            IMU.data.roll = double(imu.R)*pi/180;
            IMU.data.pitch = double(imu.P)*pi/180;
            IMU.data.yaw = double(imu.Y)*pi/180;
            IMU.data.wyaw = -1*(double(imu.Wz)-370)*(pi/180.0/3.45);...
                %double(imu.Wz);
            IMU.data.t = double( imu.t );
            IMU.tLastArrival = double(IMU.data.t);
            shm_slam_process_odometry();
        else
            %disp('flir')
            flir = msgpack('unpack', data{i});
            flimg = reshape(typecast(flir.data,'uint16'),[320,256]);
            %return;
        end
    end
    
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
    
    subplot(2,2,2);
    if isfield(LIDAR0.scan,'ranges') && ~isempty( IMU.data )
        cla;
        polar( LIDAR0.angles(:), LIDAR0.scan.ranges(:), 'r.' );
        hold on;
        xd = 10*cos(IMU.data.yaw);
        yd = 10*sin(IMU.data.yaw);
        compass(xd,yd);
    end
    
    if isempty(hFlir)
        subplot(2,2,3);
        colormap(hot);
        hFlir = imagesc( flimg' );
    else
        set(hFlir,'cdata', flimg');
    end
    
    
    
    drawnow;
end