%% Human Robot Interaction Test script
%% Copyright 2013 Stephen McGill

% Clear each time before execution
clear all;
close all;
% Add the path elements to get the mex stuff
addpath(genpath('.'))

% Listen for jpeg image data
s_img = zmq( 'subscribe', 'img' );
% Listen for omap data
s_omap = zmq( 'subscribe', 'omap' );
% Listen for octomap data
s_octo = zmq( 'subscribe', 'octo' );
% Listen for lidar data
s_lid_head = zmq( 'subscribe', 'lidar0' );
s_lid_chest = zmq( 'subscribe', 'lidar1' );
% Listen for pose data
s_pose = zmq( 'subscribe', 'pose' );
% Listen for imu data
s_imu = zmq( 'subscribe', 'imu' );


% Allocate memory for the camera image
img_w=640;
img_h=480;
thor_camera_img = zeros(img_w,img_h,3);
omap_invRes = 1/0.05;
omap_w=401;
omap_h=401;
thor_omap = 125*ones(omap_w,omap_h);
omap_xmin = -10; % When map shifts, this needs to be updated !!
omap_ymin = -10;
imu_roll = 0;
imu_pitch = 0;
imu_yaw = 0;
pose_x = 0;
pose_y = 0;
pose_a = 0;


%% Setup the figure
f = figure(1);

set(gcf,'doublebuffer','off');

% Plot the camera image
subplot(1,2,1);
p_camera = gca;
h_camera = imagesc( thor_camera_img );
axis equal
title(p_camera, 'Camera Image' );
xlim([1 img_w]);
ylim([1 img_h]);


% Plot the occupancy map
subplot(1,2,2);
p_occupancy = gca;
h_occupancy = imagesc( thor_omap,[0 255] );
axis equal
colormap(hot)
title_omap = title(p_occupancy, 'Occupancy Map', 'FontSize',12 );
xlim([1 omap_w]);
ylim([1 omap_h]);
    % Change the axis to display actual meters 
set(p_occupancy,'XTick',1:40:omap_w)
set(p_occupancy,'YTick',1:40:omap_w)
labels_right = get(p_occupancy,'xtick')-1;
labels_right = labels_right(1:ceil(size(labels_right,2)/2));
labels_left  = -fliplr(labels_right(2:end));
newlabels=[labels_left/omap_invRes labels_right/omap_invRes];
set(p_occupancy,'xticklabel',newlabels)
set(p_occupancy,'yticklabel',newlabels')
xlabel('x  ( m )')
ylabel('y  ( m )')



% Plot the robot pose
hold on;
icon_scale = floor(sqrt(omap_w*omap_h) / 10);
icon_xs = [.125 -.125 -.125] * icon_scale;
icon_ys = [0 -.10 +.10] * icon_scale;
h_pose = fill(icon_xs, icon_ys, 'g');
set(h_pose,'LineWidth',1);

%xr = xRobot*ca - yRobot * sa + pose_x;
%yr = xRobot*sa + yRobot * ca + pose_y;
set(h_pose,'XData',icon_xs+200);
set(h_pose,'YData',icon_ys+200);
hold off;

% Plot lidar readings
figure(2)
subplot(1,2,1)
h_lidar_head = plot(0,0,'b.');
xlim([-30 30])
ylim([-30 30])
title('Head Lidar')
axis square
subplot(1,2,2)
h_lidar_chest = plot(0,0,'b.');
xlim([-30 30])
ylim([-30 30])
title('Chest Lidar')
axis square

load('angles.mat')
cosangles = cos(angles);
sinangles = sin(angles);



% Draw the figure
drawnow;

%% Timing
t0 = tic;
target_fps = 30;
inv_target_fps = 1/target_fps;
dirty = 0;
counter = 0;

plot_counter = 0;

%% Begin the main loop
while 1
    counter = counter+1;
    % 1 FPS timeout
    [data,idx] = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        if s_idx==s_img % Receive camera image
            thor_camera_img = data{s};
            if( size(thor_camera_img,1)==img_w*img_h*3 )
                rgb = permute( ...
                reshape(thor_camera_img,[3,img_w,img_h]), ...
                [3 2 1]);
                set(h_camera,'Cdata', rgb);
                dirty = 1;
            end
        elseif s_idx==s_omap % Receive occupancy map data
            %disp('RECV OMAP')
            thor_omap = data{s};
            log_odds = reshape(thor_omap,[omap_h,omap_w]);
            set(h_occupancy,'Cdata', log_odds);
            dirty = 1;
        elseif s_idx==s_lid_head % Receive lidar data
            lidar_ranges = data{s};
            if( size(lidar_ranges,1)==1081*4 )
                lidar_ranges = typecast(lidar_ranges,'single');
                xlidar = lidar_ranges.*cosangles;
                ylidar = lidar_ranges.*sinangles;
                set(h_lidar_head,'XData',xlidar);
                set(h_lidar_head,'YData',ylidar);
                dirty = 1;
            end
        elseif s_idx==s_lid_chest % Receive lidar data
            lidar_ranges = data{s};
            if( size(lidar_ranges,1)==1081*4 )
                lidar_ranges = typecast(lidar_ranges,'single');
                xlidar = lidar_ranges.*cosangles;
                ylidar = lidar_ranges.*sinangles;
                set(h_lidar_chest,'XData',xlidar);
                set(h_lidar_chest,'YData',ylidar);
                dirty = 1;
            end
        elseif s_idx==s_pose % Receive pose data
            pose_data = msgpack('unpack',data{s});
            pose_a = double(pose_data.a);
            pose_x = double(pose_data.x);
            pose_y = double(pose_data.y);
            new_omap_xmin = double(pose_data.omapXmin);
            new_omap_ymin = double(pose_data.omapYmin);
            
            xshift = new_omap_xmin - omap_xmin;
            yshift = new_omap_ymin - omap_ymin;
            
            if xshift~=0 || yshift~= 0
                set(p_occupancy,'xticklabel',newlabels + xshift)
                set(p_occupancy,'yticklabel',newlabels + yshift)
                omap_xmin = new_omap_xmin;
                omap_ymin = new_omap_ymin;
            end
            
            set(title_omap,'String',...
                sprintf('Located at (%.2f %.2f) facing %.2f \n IMU rpy (%.2f %.2f %.2f)',...
                pose_x, pose_y,pose_a*180/pi, imu_roll*180/pi, imu_pitch*180/pi, imu_yaw*180/pi));
            sa = sin(pose_a);
            ca = cos(pose_a);
            
            xi = (pose_x - omap_xmin) * omap_invRes;
            yi = (pose_y - omap_ymin) * omap_invRes;
            xr = icon_xs*ca - icon_ys * sa + xi;
            yr = icon_xs*sa + icon_ys * ca + yi;
            set(h_pose,'XData',xr);
            set(h_pose,'YData',yr);
            
            set(gca, 'YDir', 'normal');
            
            dirty = 1;
        elseif s_idx==s_imu % Receive imu data
            imu = msgpack('unpack',data{s});
            imu_roll = double(imu.x);
            imu_pitch = double(imu.y);
            imu_yaw = double(imu.a);
            set(title_omap,'String',...
                sprintf('Located at (%.2f %.2f) facing %.2f \n IMU rpy (%.2f %.2f %.2f)',...
                pose_x, pose_y,pose_a*180/pi, imu_roll*180/pi, imu_pitch*180/pi, imu_yaw*180/pi));
            dirty = 1;
        end
    end
    %% Control the drawing update interval
    t_diff = toc(t0);
    if t_diff>inv_target_fps
        if dirty==1
            dirty = 0;
            drawnow;
            fprintf('Updating | FPS: %.1f\n',1/t_diff);
        else
            fprintf('No data | FPS: %.1f\n',1/t_diff);
        end
        counter = 0;
        t0 = tic;
    end
end
