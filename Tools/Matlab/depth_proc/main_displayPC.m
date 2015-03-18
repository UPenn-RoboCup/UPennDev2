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

% foldername = '/home/leebhoram/Data/LOGS_THOR/Unpacked/';
% datestamp = '02.24.2015.17.21.57'; % lab setting : door/wall/motion/terrain object

foldername = '/home/leebhoram/Data/LOGS_SC2/Unpacked/';
% datestamp = '03.12.2015.13.19.00'; % Testbed: walls (near valve)
datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)

foldername = '/home/leebhoram/Data/LOGS_SC_13-14/Unpacked/';
datestamp = '03.11.2015.13.57.00'; % Testbed: walls (hose)

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
N = length(fileSequence);
for ilog=32:N
    disp(strcat(int2str(ilog),'/(',int2str(N),')'));
    rgb_img = [];
    load(fileSequence{ilog}); 
  
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
    depthRaw = undistort_depth(depthRaw);
    D = depthRaw';
    D(D>2500) = 0;
    D(D<400) = 0;
    D = medfilt2(D,[7 7]);
   
    figure(1), imagesc(flip(D,2)); set(gca,'CLim',[0 2500]); colorbar;
   
    load(fileSequence{ilog-1}); 
    if ~isempty(rgb_img)        
        figure(2), imshow(imresize(flip(rgb_img,2),1)); 
        
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(11), 
        showPointCloud([-pcx pcy pcz], [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        
    end
    
    pause(0.1);
    
end 



