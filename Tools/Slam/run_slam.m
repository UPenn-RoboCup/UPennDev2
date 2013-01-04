
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

global SLAM OMAP POSE LIDAR0 ROBOT LIDAR IMU

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
if isempty(initSlam)
    
    disp('Initializing lidar shm...')
    if isempty(LIDAR)
        LIDAR = shm('rcm');
    end
    
    disp('Loading the Config file')
    loadConfig();
    
    SLAM.updateExplorationMap    = 0;
    SLAM.explorationUpdatePeriod = 5;
    SLAM.plannerUpdatePeriod     = 2;
    myranges = LIDAR.get_ranges();
    myranges = typecast(myranges,'single'); % 16bit precision
    myranges = myranges(1:1081);
    
    SLAM.explorationUpdateTime   = LIDAR.get_timestamp();
    SLAM.plannerUpdateTime       = LIDAR.get_timestamp();
    poseInit();
    
    SLAM.x            = POSE.xInit;
    SLAM.y            = POSE.yInit;
    SLAM.z            = POSE.zInit;
    SLAM.yaw          = POSE.data.yaw;
    SLAM.lidar0Cntr   = 0;
    SLAM.lidar1Cntr   = 0;
    
    SLAM.xOdom        = SLAM.x;
    SLAM.yOdom        = SLAM.y;
    SLAM.yawOdom      = SLAM.yaw;
    SLAM.odomChanged  = 1;
    
    SLAM.maxCost     = 100;
    SLAM.minCost     = -100;
    
    % Initialize maps
    initMapProps;
    omapInit;        %localization map
    
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
    myranges = LIDAR.get_ranges();
    myranges = typecast(myranges,'single'); % 16bit precision
    myranges = myranges(1:1081);
    
    data = {};
    data.ranges = myranges;
    data.startTime = LIDAR.get_timestamp();
    data.odometry = LIDAR.get_odom();
    data.odometry(2) = data.odometry(2)* -1; % -1 raw data from webots to MATLAB
    data.odometry(3) = data.odometry(3)* -1; % -1 raw data from webots to MATLAB
    imu = LIDAR.get_imu();
    gyro = LIDAR.get_gyro();
    IMU.data.roll = 0;
    IMU.data.pitch = 0;
    IMU.data.yaw = imu(3);
    IMU.data.wyaw = gyro(3) * -1; % -1 raw data from webots to MATLAB
    IMU.data.t = LIDAR.get_timestamp();
    IMU.tLastArrival = LIDAR.get_timestamp();
    
    %% Run SLAM Update processes
    shm_slam_process_odometry(data);
    shm_slam_process_lidar0(data);
    
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
    xd = 25*cos(POSE.data.yaw);
    yd = 25*sin(POSE.data.yaw);
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
    
    %{
    subplot(2,1,2);
    cla;
    polar( LIDAR0.angles(:),myranges.ranges(:), 'r.' );
    hold on;
    xd = max(myranges.ranges)*cos(IMU.data.yaw);
    yd = max(myranges.ranges)*sin(IMU.data.yaw);
    compass(xd,yd);
        %}
        drawnow;
        pause(.04);
end