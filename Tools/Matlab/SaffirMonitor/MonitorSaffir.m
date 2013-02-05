global MONITOR SLAM

if ismac == 1
    SHM_DIR='/tmp/boost_interprocess';
elseif isunix == 1
    SHM_DIR='/dev/shm';
end

t0=tlc;


%% Init monitor display

MONITOR = show_saffir_monitor();
%SLAM = slam(); 

t=toc(t0);
team = 0;
robot = 1;
robot = shm_robot(0,1);
nUpdate = 0;


%% Initialize slam process
%SLAM.slamStart();


while 1
  %% receive LIDAR data
  lidar_ranges=robot.hokuyo.get_ranges(); 

  %% receive IMU data
  %% receive odometry update


  %% run SLAM 

%    Horizontal LIDAR
%    slamProcessLidar0(data,name)   


  %% Update monitor
    MONITOR.update(

  %% Plan trajectory if we have an active target
  if MONITOR.target then

    %% Control the robot 

  else     %% No target

  end
end
