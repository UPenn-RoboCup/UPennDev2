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

foldername = '/home/leebhoram/Data/LOGS_Lab_0325_3/Unpacked/';
datestamp = '03.25.2015.16.40.27'; % Testbed: walls (near valve)
% datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)


%   foldername = '/home/leebhoram/Data/LOGS_Lab_0325_2/Unpacked/';
%   datestamp = '03.25.2015.16.36.47';
%foldername = '/home/leebhoram/Data/corner/Unpacked/';
%datestamp = '03.19.2015.17.53.03'; % Testbed: walls (near valve)

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 

for ilog=5:length(fileSequence)
    metad = [];
    load(fileSequence{ilog}); 
    
   
    if 1
        D = depthRaw'-20;
        D(D>4000) = 0;
        D(D<400) = 0;
        %load MASK2.mat;
        %D = D.*double(bw);
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(12), hold off;
        showPointCloud([-pcx pcy pcz]*0.001, [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); hold on;
    end
  
 
    uisetting; % See uisetting.m       size(D)
    % ui.undistortDepth = 1;
     
    ui.taskMode = 11 ;
    % average 
    [res, meta] = detectPlanes6(depthRaw, metad, ui);   
    pose = localizeCorner_v1(res,metad);
    
    
    if 0% numel(res) > 0
        d1= res{1}.Normal'*res{1}.Center
        if  numel(res) > 1
            d2= res{2}.Normal'*res{2}.Center
        end
    end
    
    ilog

    pause(0.05);  
   
end 



