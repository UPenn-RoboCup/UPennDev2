% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear all;
close all;

foldername = '/home/leebhoram/Data/LOGS_SC2/Unpacked/';
datestamp = '03.12.2015.13.19.00'; % Testbed: walls (near valve)
% datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)

%foldername = '/home/leebhoram/Data/corner/Unpacked/';
%datestamp = '03.19.2015.17.53.03'; % Testbed: walls (near valve)

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
for ilog=10:length(fileSequence)
    ilog
    metad = [];
    load(fileSequence{ilog}); 
    
    if prevts > 0
        ts = ts + metad.t - prevts;
    end
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
      
    %depthRaw = undistort_depth(depthRaw); 
    if 1
    D = depthRaw'-20;
    D(D>4000) = 0;
    D(D<400) = 0;
    load MASK2.mat;
    D = D.*double(bw);
    [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
    figure(12), hold off;
    showPointCloud([-pcx pcy pcz]*0.001, [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); hold on;
    end
    
    if 1
 
    uisetting; % See uisetting.m       size(D)
    % ui.undistortDepth = 1;
    
    metad = [];
    [res, meta] = detectPlanes5(depthRaw, metad, ui);
    
    ilog
    
    
    %%%%%%%%%%%%%%%%%%%
    % merge 
    if 1%ui.taskMode == 1 || ui.taskMode == 2 || ui.taskMode == 11
        % wall % ground
        [mergedRes, mergedMeta] = mergePlanes(res,meta);
    end
    
    % object candidates
    
    
%    toc,
%    angFromRobot(:,ilog) = metad.imu_rpy*180/pi;
%    disp(strcat('R:',num2str(angFromRobot(1,ilog)),...
%               ' P:',num2str(angFromRobot(2,ilog)),...
%               ' Y:',num2str(angFromRobot(3,ilog))));
   
     
  
   %if ilog == 53   
   % pause(0.1);  
   % end
    end
   % prevts = metad.t;   
  %  F(iLog) = getframe(gcf);    
end 



