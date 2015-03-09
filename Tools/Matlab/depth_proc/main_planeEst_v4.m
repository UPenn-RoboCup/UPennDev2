% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear all;
% close all;

% foldername = '/home/thor/Desktop/UPennDev/Tools/Matlab/depth_proc/UnpackedData/'; % change the folder
% foldername = '/home/leebhoram/matlab_workspace/DRC/Unpacked/'; % change the folder
% datestamp = '12.04.2014.09.30.46';
% datestamp = '01.07.2015.16.42.55';
% datestamp = '01.09.2015.16.42.40';
% datestamp = '01.20.2015.12.02.25';
foldername = '/home/leebhoram/Data/LOGS_THOR/Unpacked/';
datestamp = '02.24.2015.17.21.57';

%foldername = '/home/leebhoram/matlab_workspace/DRC/kinect2_on_robot/Unpacked/'; % change the folder
%datestamp = '01.23.2015.14.48.29'; % walking to the hallway
% datestamp =  '01.20.2015.12.04.49';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));

ts = 0;
prevts = 0;
for ilog=2:length(fileSequence)
    metad = [];
    load(fileSequence{ilog}); 
    
    if prevts > 0
        ts = ts + metad.t - prevts;
    end
    disp(strcat('t=',num2str(ts)));
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
    figure(3), subplot(2,1,1), imshow(imresize(flip(rgb_img,2),0.3))
    Planes = 0;
    
    if 1
    I = depthRaw;
    %I = undistort_depth(I')';
        
    uisetting; % See uisetting.m       
%    tic,
    [res, meta] = depth_proc(I, metad, ui);
%    toc,

%     angFromRobot(:,ilog) = metad.imu_rpy*180/pi;
%     disp(strcat('R:',num2str(angFromRobot(1,ilog)),...
%                ' P:',num2str(angFromRobot(2,ilog)),...
%                ' Y:',num2str(angFromRobot(3,ilog))));
   
     
    figure(3), subplot(2,1,2), view(0,0);
   %if ilog == 53
    pause(0.1);
   %end
    
    end
    prevts = metad.t;   
  %  F(iLog) = getframe(gcf);    
end 



