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

% foldername = '/home/thor/Desktop/UPennDev/Tools/Matlab/depth_proc/UnpackedData/'; % change the folder
% foldername = '/home/leebhoram/matlab_workspace/DRC/Unpacked/'; % change the folder
% datestamp = '12.04.2014.09.30.46';
% datestamp = '01.07.2015.16.42.55';
% datestamp = '01.09.2015.16.42.40';
% datestamp = '01.20.2015.12.02.25';
foldername = '/home/leebhoram/Data/LOGS_SC2/Unpacked/';
datestamp = '03.12.2015.13.19.00'; % Testbed: walls (near valve)
% datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)

%foldername = '/home/leebhoram/matlab_workspace/DRC/kinect2_on_robot/Unpacked/'; % change the folder
%datestamp = '01.23.2015.14.48.29'; % walking to the hallway
% datestamp =  '01.20.2015.12.04.49';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
for ilog=18:length(fileSequence)
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
      
    depthRaw = undistort_depth(depthRaw);
    D = depthRaw';
    D(D>3000) = 0;
    D(D<500) = 0;
    
%    figure(1), imagesc(flip(D,2)); set(gca,'CLim',[0 2500]); colorbar;
    Planes = 0;
    
   
    if 0
        D = medfilt2(D,[7 7]);
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(11), 
        showPointCloud([-pcx pcy pcz], [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    end
    
    if 1
    I = depthRaw;
    % I = undistort_depth(I')';
        
    uisetting; % See uisetting.m       
%    tic,
%    [res, meta] = depth_proc(I, metad, ui);
    [res, meta] = detectPlanes4sc(I, metad, ui);
    
     ilog
    
    %%%%%%%%%%%%%%%%%%%
    % merge 
    if 0 %ui.taskMode == 1 || ui.taskMode == 2 || ui.taskMode == 11
        % wall % ground
        for j1 = 1:numel(res)
            for j2 = 1:numel(res)
                if j1 ~= j2 &&  ~isempty(res{j1}) && ~isempty(res{j2})
                    % test normal 
                    normalInner = res{j1}.Normal'*res{j2}.Normal;
                    if normalInner > 0.9
                        % test position
                        if res{j1}.Normal'*(res{j1}.Center - res{j2}.Center) < 0.1
                            % merge 
                            
                            res{j1}.Normal = (res{j1}.Size*res{j1}.Normal+res{j2}.Size*res{j2}.Normal)/(res{j1}.Size+res{j2}.Size);
                            res{j1}.Normal = res{j1}.Normal/norm(res{j1}.Normal);
                            res{j1}.Center = (res{j1}.Size*res{j1}.Center+res{j2}.Size*res{j2}.Center)/(res{j1}.Size+res{j2}.Size);
                           
                            res{j1}.Size = res{j1}.Size+res{j2}.Size;
                            res{j1}.Points = [res{j1}.Points res{j2}.Points];
                            
                        end
                    end
                  
                end
            end
        end
    end
    
    % object candidates
    
    
%    toc,

%     angFromRobot(:,ilog) = metad.imu_rpy*180/pi;
%     disp(strcat('R:',num2str(angFromRobot(1,ilog)),...
%                ' P:',num2str(angFromRobot(2,ilog)),...
%                ' Y:',num2str(angFromRobot(3,ilog))));
   
     
  
   %if ilog == 53   
   % pause(0.1);  
   % end
    end
   % prevts = metad.t;   
  %  F(iLog) = getframe(gcf);    
end 



