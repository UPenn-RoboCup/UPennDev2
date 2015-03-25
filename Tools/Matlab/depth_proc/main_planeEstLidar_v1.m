% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: Lidar scans 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear all;
close all;

foldername = '/home/leebhoram/Data/mesh_logs/Unpacked/';
datestamp = '03.11.2015.15.30.41l'; % Testbed: walls (near valve)
datestamp = '03.11.2015.15.45.07l';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
for ilog=10:length(fileSequence)
    ilog
    metal = [];
    load(fileSequence{ilog}); 
       
    if 1   
  
    metal.flag = 1;
    [ Planes ] = detectPlaneInstances_lidar_v1( meshRaw', 3, metal);
    % [res, meta] = detectPlanes5(depthRaw, metad, ui);
    
    ilog
     
   
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



