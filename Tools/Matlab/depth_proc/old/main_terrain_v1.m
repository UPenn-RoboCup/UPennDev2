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

% %%%%%%%%
foldername = '/home/leebhoram/Data/LOGS_SC_13-14/Unpacked/';
datestamp = '03.11.2015.13.46.52'; % Testbed: example frame #78

% foldername = '/home/leebhoram/Data/Webots_log/Unpacked/';
% datestamp = '04.21.2015.10.27.27l';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
figure(13), 
figPos = get(gcf,'Position');
set(gcf,'Position', [figPos(1:2) 1024 420]);
for ilog = 3:length(fileSequence)
    ilog
    metad = [];
    load(fileSequence{ilog}); 
   % sprintf('%0.4f',metad.t)
   
   % depthRaw =  depthRaw'; % for windows log
    
    %tic,
    if 0
        D = depthRaw-20';
        D(D>4000) = 0;
        D(D<400) = 0;
        %load MASK2.mat;
        %D = D.*double(bw);
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(13), subplot(1,2,1), hold off;
        showPointCloud([-pcx pcy pcz]*0.001, [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); hold on;
        set(gca,'XTick',[],'YTick',[],'ZTick',[]);
        set(gca,'Color',0.95*ones(1,3));
        set(gca,'XColor',0.95*ones(1,3));
        set(gca,'YColor',0.95*ones(1,3));
        set(gca,'ZColor',0.95*ones(1,3));
        axis([-1 1 -1 1 0 4]);
        view(0,-85);
    end
   
    uisetting; % See uisetting.m       size(D)
    % ui.undistortDepth = 1;
     
    ui.taskMode = 4 ;
    ui.figures(3) = 3;
    % average 
    % tic,
    [res, meta] = detectPlanes7(depthRaw, metad, ui);   
    % toc
    if numel(res) == 1 && res{1}.Size > 1000
       %  pose = localizeCorner_v2(res,metad);
    end
    
   % toc,
    if 0% numel(res) > 0
        d1 = res{1}.Normal'*res{1}.Center;
        if  numel(res) > 1
            d2 = res{2}.Normal'*res{2}.Center;
        end
    end
    
    pause(0.01);  
    
%     M = getframe(gcf);   
%     writeVideo(vidObj,M);  
end 

% close(vidObj);


