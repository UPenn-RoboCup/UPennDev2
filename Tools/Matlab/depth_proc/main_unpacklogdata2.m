close all;
clear all;

DEPTH_W = 512;
DEPTH_H = 424;
RGB_W = 1920;
RGB_H = 1080;

% Set the path and names
% The unpacked data will be saved under <foldername>/Unpacked/<datestamp> 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 foldername = '/home/leebhoram/Data/LOGS_SC2/';
% foldername = '/home/leebhoram/Data/LOGS_SC2/';
% datestamp = '03.11.2015.10.18.09';
datestamp_kinect = '03.12.2015.13.15.22';
 % << 09-11 >> 
 % datestamp_kinect = '03.11.2015.08.33.08';
 % datestamp_kinect = '03.11.2015.08.34.22';
 
 % datestamp_kinect = '03.11.2015.10.20.09';
% datestamp_kinect = '03.11.2015.11.56.19';
% datestamp_kinect = '03.11.2015.11.57.52';
% datestamp_kinect = '03.11.2015.12.01.43';
% datestamp_kinect = '03.11.2015.12.03.08';
% datestamp_kinect = '03.11.2015.13.52.56';
% datestamp_kinect = '03.11.2015.13.46.52';
% datestamp_kinect = [];% '03.11.2015.14.56.17';


datestamp_lidar = [];%  '02.24.2015.17.24.03';

%foldername = '/home/leebhoram/Data/mesh_logs';
%datestamp_kinect = '01.20.2015.12.04.49';
%datestamp_lidar =  '03.11.2015.15.18.11'; %'01.23.2015.14.48.29';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

filename_depth = sprintf('k2_depth_r_%s.log',datestamp_kinect);
filename_rgb = sprintf('k2_rgb_r_%s.log', datestamp_kinect);

% search the files 
olderFolder = cd(foldername);
logFiles = dir('*.log');
cd(olderFolder);
flag_depth = find(cellfun(@(x) strcmp(x,filename_depth),{logFiles(:).name}));
flag_rgb = find(cellfun(@(x) strcmp(x,filename_rgb),{logFiles(:).name}));
flag_lidar = [];
if ~isempty(datestamp_lidar)
    filename_lidar = sprintf('mesh_r_%s.log',datestamp_lidar);
    flag_lidar = find(cellfun(@(x) strcmp(x,filename_lidar),{logFiles(:).name}));
end

% depth & rgb
if ~isempty(flag_depth) && ~isempty(flag_rgb)
    f_depth = fopen(sprintf('%s/%s', foldername,filename_depth));     
    fid = fopen(sprintf('%s/k2_depth_m_%s.log', foldername,datestamp_kinect));
    depthMeta = fread(fid,Inf,'*uint8');fclose(fid);
    depthMeta = msgpack('unpacker', depthMeta, 'uint8');

    f_rgb = fopen(sprintf('%s/%s', foldername,filename_rgb));     
    fid = fopen(sprintf('%s/k2_rgb_m_%s.log',foldername,datestamp_kinect));
    rgbMeta = fread(fid,Inf,'*uint8');fclose(fid);
    rgbMeta = msgpack('unpacker',rgbMeta,'uint8');
    
    olderFolder = cd(foldername);
    if ~exist('Unpacked','dir'),
        mkdir('Unpacked');
    end
    cd('Unpacked');
    if ~exist(datestamp_kinect,'dir'),mkdir(datestamp_kinect);end
    cd(olderFolder);
end

% lidar
if ~isempty(flag_lidar), 
    f_lidar = fopen(sprintf('%s/%s', foldername,filename_lidar));     
    fid = fopen(sprintf('%s/mesh_m_%s.log',foldername, datestamp_lidar));
    meshMeta = fread(fid, Inf, '*uchar'); 
    fclose(fid);  
    meshMeta = msgpack('unpacker', meshMeta);
    
    olderFolder = cd(foldername);
    if ~exist('Unpacked','dir'),  mkdir('Unpacked'); end
    cd('Unpacked');    
    if ~exist(strcat(datestamp_lidar,'l'),'dir'), mkdir(strcat(datestamp_lidar,'l')); end
    cd(olderFolder);
end

% depth & rgb
if  ~isempty(flag_depth) && ~isempty(flag_rgb),   
    ilog = 0;
    Nlog =  length(depthMeta);
    while ~feof(f_depth) && ilog < Nlog
        ilog = ilog + 1;    
   
        metad = depthMeta{ilog}; 
        depthRaw = fread(f_depth, [DEPTH_W, DEPTH_H], '*single');
        
        metar = rgbMeta{ilog};
        rgbJPEG = fread(f_rgb, metar.rsz, '*uint8');
        rgb_img0 = djpeg(rgbJPEG);
        rgb_img(:,:,1) = rgb_img0(:,:,1);
        rgb_img(:,:,2) = rgb_img0(:,:,2);
        rgb_img(:,:,3) = rgb_img0(:,:,3);       
         figure(2), imshow(imresize(rgb_img,0.25));       
    
        save(strcat(foldername,'/Unpacked/',datestamp_kinect,'/',sprintf('%04d.mat',ilog)),'depthRaw', 'rgb_img', 'metad', 'metar');
        depthRaw(depthRaw<500) = 0;
        depthRaw(depthRaw>4000) = 0;
        medfilt2(depthRaw',[7 7]);
        figure(1), imagesc(depthRaw'); axis equal; colorbar;
        
        
        disp(strcat('kinect :',int2str(ilog) ,'/', int2str(Nlog)));
        figure(1),
        figure(2),
        
        pause(0.2)
    end
    disp('kinect : Unpacked all!');
    fclose(f_depth);
    fclose(f_rgb);
end

% lidar
if ~isempty(flag_lidar), 
    ilog = 0;
    Nlog = length(meshMeta);
    while ~feof(f_lidar) && ilog < Nlog
        ilog = ilog + 1;    
        metal = obj{i};
        n_scanlines = metal.dims(1);
        n_returns = metal.dims(2);           
        meshRaw = fread(f_lidar, [n_returns n_scanlines], '*single')';    
        meshRaw(meshRaw>4)=0;
       % s = reshape(s, flip(o.dims));
        
        figure(1), imagesc(meshRaw);
        
        save(strcat(foldername,'/Unpacked/',datestamp_lidar,'l/',sprintf('%lidar04d.mat',ilog)),'meshRaw', 'metal');
    
        pause(0.2);   
        
        disp(strcat('lidar :',int2str(ilog) ,'/', int2str(Nlog)));
    end
    datestamp_lidar
    fclose(f_lidar);
    disp('lidar : Unpacked all!');
end


